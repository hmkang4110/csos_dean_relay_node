/*
 * Central: scan -> connect -> discover (all target svcs) -> subscribe notifiable chars
 * On notification: log hex dump
 */

/* This file header name*/
#include "ble_relay_control.h"

/* BLUETOOTH HEADERS */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* ZEPHYR KERNEL HEADERS */
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/sys/util.h>
#include <zephyr/ipc/ipc_service.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <soc.h>
#include <errno.h>

/* ZEPHYR LOGGING HEADERS */
#include <zephyr/logging/log.h>

/* ZEPHYR KCONFIG SETTINGS HEADERS */
#include <zephyr/settings/settings.h>

/* MY SERVICE UUID HEADERS === */
// #include "ble.h"
// #include "config_service.h"
// #include "grideye_service.h"
// #include "peripheral_service.h"
// #include "env_service.h"
// #include "sound_service.h"
// #include "ubinos_service.h"

#include "relay_stub_service.h"
#include "inference_service.h"


#define MAX_SUBS 24

LOG_MODULE_REGISTER(central_scan, LOG_LEVEL_INF);

/* FUNCTION PRE-DEFINITIONS */
static void reset_work_handler(struct k_work *work);
static void adv_restart_work_handler(struct k_work *work);
static void scan_restart_work_handler(struct k_work *work);
static void initiate_timeout_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(scan_restart_work, scan_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);
K_WORK_DELAYABLE_DEFINE(initiating_timeout_work, initiate_timeout_work_handler);

static void adv_start_safe(int delay_ms);
static void adv_stop_safe(void);
static void scan_start_safe(int delay_ms);
static void scan_stop_safe(void);
static void scan_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad);
static int start_discovery(struct bt_conn *conn);
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params);
static bool ad_parse_cb (struct bt_data * data, void *user_data);
static int hci_vs_write_adv_tx_power(int8_t tx_dbm);
static int hci_vs_read_adv_tx_power(int8_t *out_dbm);
static uint8_t generic_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static struct relay_session * get_session_by_hub_conn(struct bt_conn *conn);
static struct relay_session * get_session_by_dean_conn(struct bt_conn *conn);
static struct relay_session * find_empty_session(void);
static int start_spoofed_advertising(struct relay_session *session);
static void stop_spoofed_advertising(struct relay_session * session);

/* GLOBAL PARAMETER DEFINITIONS */
static uint32_t adv_backoff_ms = 200;
static uint32_t scan_backoff_ms = 200;
static uint32_t initiate_start_ms = 0;
#define BACKOFF_CAP 2000

static atomic_t adv_on;
static atomic_t scan_on;
static atomic_t initiating;

static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subs[MAX_SUBS];
static size_t subs_cnt;
static uint16_t h_remote_rawdata;
static uint16_t h_remote_seq_result;
static uint16_t h_remote_debug_string;

struct adv_match_ctx
{
    bool name_match;
    char found_name[20];    // BT_GAP_MAX_NAME_LEN
};
static struct bt_conn *central_conn;
static struct bt_conn *central_pending;
static struct bt_conn *peripheral_conn;

struct relay_session g_relay_sessions[MAX_RELAY_SESSIONS] = {0};

/* BLE CENTRAL PARAMETERS */
#define BLE_SCAN_INTERVAL 80    /* 50 ms */
#define BLE_SCAN_WINDOW   80    /* 50 ms */
#define BLE_SCAN_ACTIVE_SLOW BT_LE_SCAN_PARAM(BT_LE_SCAN_TYPE_ACTIVE, \
                                               BT_LE_SCAN_OPT_NONE,   \
                                               BLE_SCAN_INTERVAL,     \
                                               BLE_SCAN_WINDOW)
#define ADV_PACKET_STR_LEN          30
#define MAC_ADDR_STR_LEN            17
#define BT_DEVICE_CONNECT_LIST_NUM  1

/* BLE PERIPHERAL PARAMETERS */
#define BLE_DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define BLE_DEVICE_NAME_LEN (sizeof(BLE_DEVICE_NAME) - 1)

static const struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN),
};
static const struct bt_data scan_rsp_data[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BASE_SERVICE_VAL),
};

/* BLE COMMON FUNCTION, PARAMETERS */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* KERNEL WORK HANDLERS */
static void scan_restart_work_handler(struct  k_work *work)
{
    if (atomic_get(&scan_on) == 1) {
        return;
    }

    LOG_INF("[SCAN] scan restart work handler");
    int err = bt_le_scan_start(BLE_SCAN_ACTIVE_SLOW, scan_device_found);

    if (err == -EALREADY) {
        atomic_set(&scan_on, 1);
        scan_backoff_ms = 200;
        return;
    }

    if (err == -EBUSY) {
        scan_backoff_ms = MIN(scan_backoff_ms * 2, BACKOFF_CAP);
        k_work_reschedule(&scan_restart_work, K_MSEC(scan_backoff_ms));
        return;
    }

    if (!err) {
        atomic_set(&scan_on, 1);
        scan_backoff_ms = 200;
    } else {
        LOG_WRN("[SCAN] bt_le_scan_start failed (err %d), retry", err);
        k_work_reschedule(&scan_restart_work, K_MSEC(300));
    }
}

static void adv_restart_work_handler(struct k_work *work)
{
    int err = 0;

    if (atomic_get(&adv_on)) {
        return;
    }

    LOG_INF("[ADV] adv restart work handler");

    // err = bt_le_adv_start(BT_LE_ADV_CONN,
    //                       adv_data,
    //                       ARRAY_SIZE(adv_data),
    //                       scan_rsp_data,
    //                       ARRAY_SIZE(scan_rsp_data));
    // if (err == -EALREADY) {
    //     LOG_INF("[ADV] adv already on");
    //     atomic_set(&adv_on, 1);
    //     adv_backoff_ms = 200;
    //     return;
    // }

    // if (err == -EBUSY) {
    //     LOG_WRN("[ADV] adv start busy, backoff %d ms", adv_backoff_ms);
    //     adv_backoff_ms = MIN(adv_backoff_ms * 2, BACKOFF_CAP);
    //     k_work_reschedule(&adv_restart_work, K_MSEC(adv_backoff_ms));
    //     return;
    // }

    if (!err) {
        atomic_set(&adv_on, 1);
        adv_backoff_ms = 200;
        scan_start_safe(1000);
    } else {
        LOG_WRN("[ADV] bt_le_adv_start failed (err %d), retry", err);
        k_work_reschedule(&adv_restart_work, K_MSEC(300));
    }
}

static void initiate_timeout_work_handler(struct k_work *work)
{
    if (atomic_get(&initiating) == 1) {
        LOG_WRN("[INITIATE] create timeout -> cancel");
        bt_le_create_conn_cancel();
        atomic_set(&initiating, 0);
        if (central_pending)
        {
            bt_conn_unref(central_pending);
            central_pending = NULL;
        }
        scan_start_safe(300);
    }
}

/* FUNCTION DEFINITIONS */
static void scan_start_safe(int delay_ms)
{
    k_work_reschedule(&scan_restart_work, K_MSEC(delay_ms));
}

static void scan_stop_safe(void)
{
    int err = bt_le_scan_stop();

    if (err == -EALREADY) {
        atomic_set(&scan_on, 0);
        return;
    }

    if (!err) {
        atomic_set(&scan_on, 0);
    } else {
        LOG_WRN("[SCAN] bt_le_scan_stop failed (err %d)", err);
    }
}

static void adv_start_safe(int delay_ms)
{
    k_work_reschedule(&adv_restart_work, K_MSEC(delay_ms));
}

static void adv_stop_safe(void)
{
    int err = bt_le_adv_stop();

    if (err == -EALREADY) {
        atomic_set(&adv_on, 0);
        return;
    }

    if (!err) {
        atomic_set(&adv_on, 0);
    } else {
        LOG_WRN("[ADV] bt_le_adv_stop failed (err %d)", err);
    }
}

static int hci_vs_write_adv_tx_power(int8_t tx_dbm)
{
    struct bt_hci_cp_vs_write_tx_power_level *cp;
    struct net_buf *buf, *rsp = NULL;

    buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, sizeof(*cp));
    if (!buf) {
        return -ENOMEM;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle_type    = BT_HCI_VS_LL_HANDLE_TYPE_ADV; /* 광고 세트 */
    cp->handle         = sys_cpu_to_le16(0);           /* set #0 (legacy adv) */
    cp->tx_power_level = tx_dbm;

    int err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL, buf, &rsp);
    if (rsp) {
        net_buf_unref(rsp);
    }
    return err;
}

static int hci_vs_read_adv_tx_power(int8_t *out_dbm)
{
    struct bt_hci_cp_vs_read_tx_power_level *cp;
    struct bt_hci_rp_vs_read_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;

    buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL, sizeof(*cp));
    if (!buf) {
        return -ENOMEM;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle_type = BT_HCI_VS_LL_HANDLE_TYPE_ADV;
    cp->handle      = sys_cpu_to_le16(0);

    int err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL, buf, &rsp);
    if (err) {
        return err;
    }

    rp = (void *)rsp->data;
    *out_dbm = (int8_t)rp->tx_power_level;
    net_buf_unref(rsp);
    return 0;
}

/** @brief SCAN result callback function. */
static void scan_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    int err;

    if (find_empty_session() == NULL)
    {
        return;
    }
    struct bt_conn *tmp_conn = NULL;

    /* Connect only with connectable adv/scan rsp packet */
    if (type != BT_GAP_ADV_TYPE_ADV_IND &&
        type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
        type != BT_GAP_ADV_TYPE_EXT_ADV &&
        type != BT_GAP_ADV_TYPE_SCAN_RSP) {
        return;
    }

    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    // LOG_DBG("[DEVICE]: %s (RSSI %d)", addr_str, rssi);

    /* for debugging: 이름 매칭 */
    struct adv_match_ctx ctx = {0};

    if (ad && ad->len) {
        bt_data_parse(ad, ad_parse_cb, &ctx);
    }

    if (!ctx.name_match) {
        return;
    }

    /* Connect only to devices in close proximity */
    if (rssi < -95) {
        return;
    }

    LOG_INF("[MATCH] name=\"%s\" from %s (RSSI %d)", ctx.found_name, addr_str, rssi);

    scan_stop_safe();
    atomic_set(&initiating, 1);
    initiate_start_ms = k_uptime_get_32();
    // k_work_reschedule(&initiating_timeout_work, K_SECONDS(10));

    err = bt_conn_le_create(addr,
                            BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT,
                            &tmp_conn);
    if (err) 
    {
        LOG_WRN("[DEVICE FOUND] Create connection to %s failed (err %d)", addr_str, err);
        if (tmp_conn) 
        {
            bt_conn_unref(tmp_conn);
        }
        atomic_set(&initiating, 0);
        scan_start_safe(300);
        return;
    }
    else 
    {
        central_pending = bt_conn_ref(tmp_conn);
        bt_conn_unref(tmp_conn);
        LOG_INF("[DEVICE FOUND] Creating connection to %s | [%s]", addr_str, ctx.found_name);
    }
}

static bool ad_parse_cb (struct bt_data * data, void *user_data)
{
    struct adv_match_ctx *ctx = (struct adv_match_ctx *)user_data;

    switch (data->type) {
    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_NAME_SHORTENED: {
        size_t n = MIN((size_t)data->data_len, sizeof(ctx->found_name) - 1);
        memcpy(ctx->found_name, data->data, n);
        ctx->found_name[n] = '\0';

        char target_peripheral_name[20] = "DE&N";

        if (strcmp(ctx->found_name, target_peripheral_name) == 0) {
            ctx->name_match = true;
            LOG_DBG("[AD] matched device name: %s", ctx->found_name);
        }
        break;
    }
    default:
        break;
    }
    return true;
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    /* 1) 탐색 종료 조건 */
    if (!attr) {
        LOG_INF("[DISCOVER] type %u complete", params->type);
        memset(params, 0, sizeof(*params));   /* 이 discover 작업은 끝 */
        return BT_GATT_ITER_STOP;
    }

    /* 2) 우리는 CHARACTERISTIC 탐색만 사용 중 */
    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chrc = attr->user_data;
        uint16_t decl_handle  = attr->handle;          /* Characteristic Declaration */
        uint16_t value_handle = chrc->value_handle;    /* Characteristic Value */

        LOG_DBG("[DISCOVER] char decl=0x%04x val=0x%04x props=0x%02x",
                decl_handle, value_handle, chrc->properties);

        /* 2-1) Notify 지원하는 Characteristic 인가? */
        if (chrc->properties & BT_GATT_CHRC_NOTIFY) {

            if (chrc->properties & BT_GATT_CHRC_NOTIFY)
            {
                if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_RAWDATA))
                {
                    h_remote_rawdata = value_handle;
                    LOG_INF("[DISCOVER] found INFERENCE_RAWDATA char at 0x%04x", value_handle);
                }
                else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT))
                {
                    h_remote_seq_result = value_handle;
                    LOG_INF("[DISCOVER] found INFERENCE_SEQ_ANAL_RESULT char at 0x%04x", value_handle);
                }
                else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_DEBUG_STRING))
                {
                    h_remote_debug_string = value_handle;
                    LOG_INF("[DISCOVER] found INFERENCE_DEBUG_STRING char at 0x%04x", value_handle);
                }
            }

            if (subs_cnt >= MAX_SUBS) {
                LOG_WRN("[DISCOVER] subscribe table full, skip");
                return BT_GATT_ITER_CONTINUE;
            }

            struct bt_gatt_subscribe_params *sub = &subs[subs_cnt];
            memset(sub, 0, sizeof(*sub));

            /* 단순 가정: CCCD = value_handle + 1 */
            sub->ccc_handle   = (uint16_t)(value_handle + 1);
            sub->value_handle = value_handle;
            sub->value        = BT_GATT_CCC_NOTIFY;
            sub->notify       = generic_notify_cb;

            int err = bt_gatt_subscribe(conn, sub);
            if (err && err != -EALREADY) {
                LOG_WRN("[DISCOVER] subscribe failed: val=0x%04x ccc=0x%04x err=%d",
                        sub->value_handle, sub->ccc_handle, err);
                memset(sub, 0, sizeof(*sub));
            } else {
                LOG_INF("[DISCOVER] subscribed: val=0x%04x ccc=0x%04x (idx=%u)",
                        sub->value_handle, sub->ccc_handle, (unsigned)subs_cnt);
                subs_cnt++;
            }
        }

        return BT_GATT_ITER_CONTINUE;
    }

    /* 지금은 다른 type 을 쓰지 않지만, 확장 대비 */
    LOG_DBG("[DISCOVER] unsupported discover type=%u at handle=0x%04x",
            params->type, attr->handle);
    return BT_GATT_ITER_CONTINUE;
}

static int start_discovery(struct bt_conn *conn)
{
    int err;

    memset(&discover_params, 0, sizeof(discover_params));

    /* 서비스 UUID를 모른다는 가정 → ATT 전체 범위에서
     * 모든 Characteristic 을 한 번 훑는다.
     */
    discover_params.uuid         = NULL; /* 모든 캐릭터리스틱 */
    discover_params.func         = discover_func;
    discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    err = bt_gatt_discover(conn, &discover_params);
    if (err) {
        LOG_ERR("Discover failed (err %d)", err);
        return err;
    }

    LOG_INF("Discover started (all characteristics)");
    return 0;
}

static uint8_t generic_notify_cb(struct bt_conn *conn,
                                 struct bt_gatt_subscribe_params *params,
                                 const void *data,
                                 uint16_t length)
{
    int err = 0;
    if (!data) {
        LOG_INF("[NOTIFY] Unsubscribed from handle %u", params->value_handle);
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    uint16_t handle = params->value_handle;
    if (handle == h_remote_rawdata && length == INFERENCE_RESULT_PACKET_SIZE)
    {
        if (is_inference_notify_enabled() == false)
        {
            return BT_GATT_ITER_CONTINUE;
        }
        else 
        {
            err = bt_inference_rawdata_send((uint8_t *)data);
            if (err)
            {
                LOG_WRN("[RELAY] INFERENCE_RAWDATA send failed (err %d)", err);
            }
        }
    }
    else if (handle == h_remote_seq_result)
    {
        // int err = bt_inference_seq_result_send((uint8_t *)data, length);
        if (err)
        {
            LOG_WRN("[RELAY] INFERENCE_SEQ_ANAL_RESULT send failed (err %d)", err);
        }
    }
    else if (handle == h_remote_debug_string)
    {
        if (is_inference_debug_string_notify_enabled() == false)
        {
            return BT_GATT_ITER_CONTINUE;
        }
        else
        {
            err = bt_inference_debug_string_send((uint8_t *)data, length);
            if (err)
            {
                LOG_WRN("[RELAY] INFERENCE_DEBUG_STRING send failed (err %d)", err);
            }
        }
    }
    else 
    {
        LOG_WRN("[NOTIFY] Unknown handle=0x%04x len=%u", handle, length);
    }

    // const uint8_t *p = data;
    // char buf[128];
    // int off = 0;

    // off += snprintk(buf + off, sizeof(buf) - off,
    //                 "[NOTIFY] handle=%u len=%u data=",
    //                 params->value_handle, length);

    // for (uint16_t i = 0; i < length && off < (int)sizeof(buf) - 3; i++) {
    //     off += snprintk(buf + off, sizeof(buf) - off, "%02X ", p[i]);
    // }

    // LOG_INF("%s", buf);

    return BT_GATT_ITER_CONTINUE;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    int err = 0;
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    bt_conn_get_info(conn, &info);

    /* connection failed */
    if (conn_err) {

        if (info.role == BT_CONN_ROLE_CENTRAL) {
            /* CENTRAL: DEAN node 연결 실패 */
            LOG_WRN("[CONNECTED] Failed to connect to peripheral %s (err %u)", addr, conn_err);

            if (central_pending == conn) {
                bt_conn_unref(central_pending);
                central_pending = NULL;
            }

            atomic_set(&initiating, 0);
            atomic_set(&scan_on, 0);
            scan_start_safe(300);
            return;
        }
        else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
            /* PERIPHERAL: SLIMHUB 가 나한테 붙으려다 실패 */
            LOG_WRN("[CONNECTED] Failed to accept central %s (err %u)", addr, conn_err);

            if (peripheral_conn == conn) {
                bt_conn_unref(peripheral_conn);
                peripheral_conn = NULL;
            }

            /* 광고 다시 */
            atomic_set(&adv_on, 0);
            adv_start_safe(300);
            return;
        }
    }
    else {
        /* connection success */

        if (info.role == BT_CONN_ROLE_CENTRAL) {
            /* relay node 가 CENTRAL 로서 DEAN node 에 붙은 상황 */

            // 빈 세션을 찾아 할당
            struct relay_session *session = find_empty_session();
            if (session)
            {
                session->is_active = true;
                session->conn_dean = bt_conn_ref(conn);

                LOG_INF("[CONNECTED] Session %d DEAN connected! Starting Proxy advertising...", session->index);
                start_discovery(conn);
                err = start_spoofed_advertising(session);
                if (err)
                {
                    LOG_ERR("[CONNECTED] start_spoofed_advertising failed (err %d)", err);
                }
                else
                {
                    LOG_INF("[CONNECTED] Proxy advertising started for session %d", session->index);
                    err = hci_vs_write_adv_tx_power(20);
                    if (err == 0)
                    {
                        int8_t eff;
                        if (hci_vs_read_adv_tx_power(&eff) == 0)
                        {
                            LOG_INF("[HCI] ADV TX set=20 dBm, effective=%d dBm%s",
                                    eff, (eff > 8) ? "  <-- FEM-updated" : "");
                        }
                        else
                        {
                            LOG_ERR("[HCI] READ adv TX failed");
                        }
                    }
                    else
                    {
                        LOG_ERR("[HCI] WRITE adv TX(20) failed (%d)", err);
                    }
                }
            }
            else
            {
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }
        }
        else if (info.role == BT_CONN_ROLE_PERIPHERAL) 
        {
            /* relay node 가 PERIPHERAL 로서 SLIMHUB 에 붙은 상황 */

            bool mapped = false;
            for (int i=0; i < MAX_RELAY_SESSIONS; i++)
            {
                if (g_relay_sessions[i].is_active && g_relay_sessions[i].conn_dean != NULL)
                {
                    g_relay_sessions[i].conn_slimhub = bt_conn_ref(conn);
                    mapped = true;
                    LOG_INF("[CONNECTED] Session %d SLIMHUB connected!", g_relay_sessions[i].index);
                    break;
                }
            }

            if (!mapped) {
                LOG_WRN("[CONNECTED] No active session found for SLIMHUB connection");
                bt_conn_disconnect(conn, BT_HCI_ERR_UNACCEPT_CONN_PARAM);
            }

            if (!peripheral_conn) {
                peripheral_conn = bt_conn_ref(conn);
            }

            LOG_INF("[CONNECTED] Connection established as PERIPHERAL with central %s", addr);
            atomic_set(&adv_on, 0);
        }
    }

    LOG_INF("[CONNECTED] Connected: %s (role=%s)",
            addr,
            (info.role == BT_CONN_ROLE_CENTRAL) ? "CENTRAL" : "PERIPHERAL");

    atomic_set(&initiating, 0);
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct relay_session *sess;
    char addr[BT_ADDR_LE_STR_LEN];

    /* 로그용 주소 변환 (옵션) */
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    /* ------------------------------------------------------------------
     * CASE 1: 끊어진 녀석이 DEAN Node (Central로 연결했던 것) 인지 확인
     * ------------------------------------------------------------------ */
    sess = get_session_by_dean_conn(conn);
    if (sess) {
        LOG_INF("[SESSION %d] DEAN Disconnected (%s, reason %u)", sess->index, addr, reason);
        LOG_INF("  -> HARD RESET: Clearing session and stopping proxy adv.");

        /* 1. 짝꿍 SLIMHUB가 있다면 강제로 끊어준다 (가상 연결이므로 본체가 없으면 무의미) */
        if (sess->conn_slimhub) {
            LOG_INF("  -> Disconnecting associated SLIMHUB");
            bt_conn_disconnect(sess->conn_slimhub, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            
            /* SLIMHUB의 disconnected 콜백이 또 불릴 수 있으므로 미리 NULL 처리 및 unref */
            bt_conn_unref(sess->conn_slimhub);
            sess->conn_slimhub = NULL;
        }

        /* 2. 이 DEAN을 흉내내던 가짜 Advertising 중지 */
        stop_spoofed_advertising(sess);

        /* 3. DEAN 연결 객체 해제 */
        bt_conn_unref(sess->conn_dean);
        sess->conn_dean = NULL;

        /* 4. 세션 정보 초기화 (이제 이 슬롯은 빈 자리가 됨) */
        memset(sess, 0, sizeof(struct relay_session));

        /* 5. 빈 자리가 생겼으니, 다른 DEAN을 찾기 위해 스캔 재시작 */
        scan_start_safe(300);
        
        return;
    }

    /* ------------------------------------------------------------------
     * CASE 2: 끊어진 녀석이 SLIMHUB (Peripheral로 연결됐던 것) 인지 확인
     * ------------------------------------------------------------------ */
    sess = get_session_by_hub_conn(conn);
    if (sess) {
        LOG_INF("[SESSION %d] SLIMHUB Disconnected (%s, reason %u)", sess->index, addr, reason);
        LOG_INF("  -> SOFT RESET: Restarting proxy adv to lure SLIMHUB back.");

        /* 1. SLIMHUB 연결 객체 해제 */
        bt_conn_unref(sess->conn_slimhub);
        sess->conn_slimhub = NULL;

        /* 2. DEAN은 아직 붙어있으므로, SLIMHUB가 다시 붙을 수 있게 Advertising 재시작 */
        /* 주의: sess->adv 핸들은 살아있으므로 start만 하면 됨 */
        if (sess->adv) {
            int err = bt_le_ext_adv_start(sess->adv, BT_LE_EXT_ADV_START_DEFAULT);
            if (err) {
                LOG_ERR("Failed to restart adv for session %d (err %d)", sess->index, err);
            } else {
                LOG_INF("  -> Proxy Advertising restarted.");
            }
        }
        return;
    }

    /* ------------------------------------------------------------------
     * CASE 3: 세션에 등록되지 않은 기타 연결 해제 (예: 연결 시도 중 실패 등)
     * ------------------------------------------------------------------ */
    LOG_INF("[DISCONNECTED] Unmapped connection %s (reason %u). Ignoring.", addr, reason);
    
    /* 여기서 conn unref를 하지 않습니다. 
     * (우리가 세션에 등록하며 ref를 증가시킨 적이 없으므로, Zephyr 스택이 알아서 정리함) */
}

static struct relay_session * get_session_by_hub_conn(struct bt_conn *conn)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++)
    {
        if (g_relay_sessions[i].is_active && g_relay_sessions[i].conn_slimhub == conn)
        {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

static struct relay_session * get_session_by_dean_conn(struct bt_conn *conn)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++)
    {
        if (g_relay_sessions[i].is_active && g_relay_sessions[i].conn_dean == conn)
        {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

static struct relay_session * find_empty_session(void)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++)
    {
        if (!g_relay_sessions[i].is_active)
        {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

static int start_spoofed_advertising(struct relay_session *session)
{
    int err;
    char name[30];

    err = bt_id_create(&session->adv_id, NULL);
    struct bt_le_adv_param adv_param = {
        .id = session->adv_id,
        .options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    err = bt_le_ext_adv_create(&adv_param, NULL, &session->adv);
    if (err)
    {
        LOG_ERR("[SPOOF ADV] ext adv create failed (err %d)", err);
        return err;
    }
    return bt_le_ext_adv_start(session->adv, BT_LE_EXT_ADV_START_DEFAULT);
}

static void stop_spoofed_advertising(struct relay_session * session)
{
    if (session->adv)
    {
        bt_le_ext_adv_stop(session->adv);
        bt_le_ext_adv_delete(session->adv);
        session->adv = NULL;
    }
}



/* External Called function*/
int ble_relay_control_start(void)
{
    int err = 0;

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BLE init failed (err %d)", err);
        return err;
    } else {
        LOG_INF("BLE init success");
        err = settings_load();
        if (err) {
            LOG_WRN("Settings load failed (err %d)", err);
        }
        k_sleep(K_MSEC(500));
    }

    adv_start_safe(0);

    return err;
}
