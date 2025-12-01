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

/* Service Headers for UUIDs */
#include "ble.h"
#include "config_service.h"
#include "grideye_service.h"
#include "peripheral_service.h"
#include "env_service.h"
#include "sound_service.h"
#include "relay_stub_service.h"
#include "inference_service.h"


#define MAX_SUBS 24

LOG_MODULE_REGISTER(central_scan, LOG_LEVEL_INF);

/* FUNCTION PRE-DEFINITIONS */
static void reset_work_handler(struct k_work *work);
static void adv_restart_work_handler(struct k_work *work);
static void scan_restart_work_handler(struct k_work *work);

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

static void start_cloned_advertising(void);

static struct bt_gatt_discover_params discover_params;

K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(scan_restart_work, scan_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);

/* GLOBAL PARAMETER DEFINITIONS */
static uint32_t adv_backoff_ms = 200;
static uint32_t scan_backoff_ms = 200;
#define BACKOFF_CAP 2000

static atomic_t adv_on;
static atomic_t scan_on;
static atomic_t initiating;

/* Captured Target Data */
static char captured_name[BT_GAP_ADV_MAX_NAME_LEN + 1];
static struct bt_uuid_128 captured_uuids[5]; // Store a few captured UUIDs
static int captured_uuid_count = 0;

/* Remote Handles for Forwarding */
static struct bt_gatt_subscribe_params subs[MAX_SUBS];
static size_t subs_cnt;

// Read/Notify handles
static uint16_t h_remote_rawdata;
static uint16_t h_remote_seq_result;
static uint16_t h_remote_debug_string;

// Write handles (Upstream Relay)
static uint16_t h_remote_write_rawdata; // For INFERENCE_RAWDATA (Write/Read/Notify)
static uint16_t h_remote_file_transfer;
static uint16_t h_remote_device_name;
static uint16_t h_remote_location;
static uint16_t h_remote_grideye_prediction;

struct adv_match_ctx
{
    bool name_match;
    char found_name[BT_GAP_ADV_MAX_NAME_LEN + 1];
    struct bt_uuid_128 uuids[5];
    int uuid_count;
};
static struct bt_conn *central_conn;
static struct bt_conn *central_pending;
static struct bt_conn *peripheral_conn;

/* Extended Advertising Handle */
static struct bt_le_ext_adv *adv;

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
// Initial default name before cloning
#define BLE_DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define BLE_DEVICE_NAME_LEN (sizeof(BLE_DEVICE_NAME) - 1)

static struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN),
};

static struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_BASE_SERVICE_VAL),
};

/* BLE COMMON FUNCTION, PARAMETERS */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* KERNEL WORK HANDLERS */
static void reset_work_handler(struct k_work *work)
{
    LOG_INF("[RESET] System Logic Reset - Starting SCAN Only");

    /* Stop everything first to ensure clean state */
    scan_stop_safe();
    adv_stop_safe();

    if (central_pending) {
        bt_conn_unref(central_pending);
        central_pending = NULL;
    }

    if (atomic_get(&initiating)) {
        int err = bt_le_create_conn_cancel();
        if (err && err != -EALREADY) {
            LOG_WRN("[RESET] Create conn cancel failed (err %d)", err);
        }
    }

    atomic_set(&initiating, 0);

    /* Start Scanning ONLY. Advertising waits for connection & cloning. */
    scan_start_safe(100);
}

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
    /* Handled by Extended Advertising API now, but keeping safe guard structure */
    if (atomic_get(&adv_on)) {
        return;
    }

    // We expect start_cloned_advertising to be called explicitly
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
    // Not using legacy adv start anymore for the relay logic
    // But keeping it as placeholder if needed
}

static void adv_stop_safe(void)
{
    if (adv) {
        int err = bt_le_ext_adv_stop(adv);
        if (!err) {
            atomic_set(&adv_on, 0);
            LOG_INF("[ADV] Extended Advertising Stopped");
        }
    } else {
        bt_le_adv_stop(); // Fallback
        atomic_set(&adv_on, 0);
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
    cp->handle_type    = BT_HCI_VS_LL_HANDLE_TYPE_ADV;
    cp->handle         = sys_cpu_to_le16(0);
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

    if (central_pending || central_conn) {
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

    /* Capture Data for Cloning */
    memcpy(captured_name, ctx.found_name, sizeof(captured_name));
    captured_uuid_count = ctx.uuid_count;
    for(int i=0; i<ctx.uuid_count; i++) {
        memcpy(&captured_uuids[i], &ctx.uuids[i], sizeof(struct bt_uuid_128));
    }

    scan_stop_safe();
    atomic_set(&initiating, 1);

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
            // LOG_DBG("[AD] matched device name: %s", ctx->found_name);
        }
        break;
    }
    case BT_DATA_UUID128_ALL:
    case BT_DATA_UUID128_SOME: {
        // Capture UUIDs (limit to 5)
        int count = data->data_len / 16;
        for (int i = 0; i < count && ctx->uuid_count < 5; i++) {
            struct bt_uuid_128 *u = (struct bt_uuid_128 *)&ctx->uuids[ctx->uuid_count];
            // Initialize the UUID structure properly
            u->uuid.type = BT_UUID_TYPE_128;
            memcpy(u->val, data->data + (i * 16), 16);
            ctx->uuid_count++;
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

        /* Discovery Finished - Start Cloning Advertising */
        start_cloned_advertising();

        return BT_GATT_ITER_STOP;
    }

    /* 2) 우리는 CHARACTERISTIC 탐색만 사용 중 */
    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chrc = attr->user_data;
        uint16_t decl_handle  = attr->handle;          /* Characteristic Declaration */
        uint16_t value_handle = chrc->value_handle;    /* Characteristic Value */

        // LOG_DBG("[DISCOVER] char decl=0x%04x val=0x%04x props=0x%02x", decl_handle, value_handle, chrc->properties);

        /* Capture Write Handles */
        if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_RAWDATA)) {
            h_remote_write_rawdata = value_handle; // For writing
            h_remote_rawdata = value_handle;       // For notification check
            LOG_INF("[DISCOVER] found INFERENCE_RAWDATA (Write/Notif) at 0x%04x", value_handle);
        }
        else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_FILE_TRANSFER)) {
            h_remote_file_transfer = value_handle;
            LOG_INF("[DISCOVER] found FILE_TRANSFER (Write) at 0x%04x", value_handle);
        }
        else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_DEVICE_NAME)) {
            h_remote_device_name = value_handle;
            LOG_INF("[DISCOVER] found DEVICE_NAME (Write) at 0x%04x", value_handle);
        }
        else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_LOCATION)) {
            h_remote_location = value_handle;
            LOG_INF("[DISCOVER] found LOCATION (Write) at 0x%04x", value_handle);
        }
        else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_GRIDEYE_PREDICTION)) {
            h_remote_grideye_prediction = value_handle;
            LOG_INF("[DISCOVER] found GRIDEYE_PREDICTION (Write) at 0x%04x", value_handle);
        }
        else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT)) {
            h_remote_seq_result = value_handle;
            LOG_INF("[DISCOVER] found SEQ_ANAL_RESULT at 0x%04x", value_handle);
        }
        else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_DEBUG_STRING)) {
            h_remote_debug_string = value_handle;
            LOG_INF("[DISCOVER] found DEBUG_STRING at 0x%04x", value_handle);
        }

        /* Subscribe to Notifications */
        if (chrc->properties & BT_GATT_CHRC_NOTIFY) {

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
                // LOG_INF("[DISCOVER] subscribed: val=0x%04x ccc=0x%04x (idx=%u)", sub->value_handle, sub->ccc_handle, (unsigned)subs_cnt);
                subs_cnt++;
            }
        }

        return BT_GATT_ITER_CONTINUE;
    }

    return BT_GATT_ITER_CONTINUE;
}

static int start_discovery(struct bt_conn *conn)
{
    int err;

    memset(&discover_params, 0, sizeof(discover_params));

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
        err = bt_inference_rawdata_send((uint8_t *)data);
        if (err)
        {
            LOG_WRN("[RELAY] INFERENCE_RAWDATA send failed (err %d)", err);
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
        err = bt_inference_debug_string_send((uint8_t *)data, length);
        if (err)
        {
            LOG_WRN("[RELAY] INFERENCE_DEBUG_STRING send failed (err %d)", err);
        }
    }

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
            start_cloned_advertising();
            return;
        }
    }
    else {
        /* connection success */

        if (info.role == BT_CONN_ROLE_CENTRAL) {
            /* relay node 가 CENTRAL 로서 DEAN node 에 붙은 상황 */

            if (central_pending == conn) {
                /* 연결 완료 → active conn 으로 승격 */
                central_conn = central_pending;
                central_pending = NULL;
            }
            else if (!central_conn) {
                /* 혹시 pending 없이 콜백이 온 경우 방어적으로 ref 확보 */
                central_conn = bt_conn_ref(conn);
            }

            LOG_INF("[CONNECTED] Connection established as CENTRAL to peripheral %s", addr);

            // Start Discovery immediately
            err = start_discovery(central_conn);
            if (err) {
                LOG_WRN("[CONNECTED] start discovery error : %d", err);
                bt_conn_disconnect(central_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            }

            atomic_set(&initiating, 0);
        }
        else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
            /* relay node 가 PERIPHERAL 로서 SLIMHUB 에 붙은 상황 */

            if (!peripheral_conn) {
                peripheral_conn = bt_conn_ref(conn);
            }

            LOG_INF("[CONNECTED] Connection established as PERIPHERAL with central %s", addr);
            atomic_set(&adv_on, 0);
        }
    }
}


static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    struct bt_conn_info info;
    int err;

    if (!conn) {
        LOG_WRN("[DISCONNECTED] conn == NULL (reason %u)", reason);
        return;
    }

    err = bt_conn_get_info(conn, &info);
    if (err) {
        bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
        LOG_INF("[DISCONNECTED] Disconnected from %s (reason %u), but get_info failed (%d)",
                addr, reason, err);
        return;
    }

    bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));

    if (info.role == BT_CONN_ROLE_PERIPHERAL) {
        /* relay node 가 PERIPHERAL 로서 SLIMHUB 에 붙어 있던 연결이 끊어진 경우 */
        LOG_INF("[DISCONNECTED] Central %s disconnected (reason %u) -> restart advertising",
                addr, reason);

        if (peripheral_conn == conn) {
            bt_conn_unref(peripheral_conn);
            peripheral_conn = NULL;
        }

        atomic_set(&adv_on, 0);
        start_cloned_advertising();
    }
    else if (info.role == BT_CONN_ROLE_CENTRAL) {
        /* relay node 가 CENTRAL 로서 DEAN node 에 붙어 있던 연결이 끊어진 경우 */
        LOG_INF("[DISCONNECTED] Peripheral %s disconnected (reason %u) -> restart scanning",
                addr, reason);

        if (central_conn == conn) {
            bt_conn_unref(central_conn);
            central_conn = NULL;
        }
        if (central_pending == conn) {
            bt_conn_unref(central_pending);
            central_pending = NULL;
        }

        atomic_set(&initiating, 0);
        memset(subs, 0, sizeof(subs));
        subs_cnt = 0;

        // Reset advertising stop logic if we lost connection to DEAN?
        // Requirement says "Boot: Start Scanning ONLY. Do NOT start advertising yet."
        // So if we lose DEAN, we should stop advertising and scan for DEAN again.
        adv_stop_safe();

        atomic_set(&scan_on, 0);
        scan_start_safe(300);
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

    /* Schedule the reset work to start multi-role logic */
    k_work_reschedule(&reset_work, K_NO_WAIT);

    return err;
}

/* New Logic: Cloned Advertising */
static void start_cloned_advertising(void)
{
    int err;
    if (atomic_get(&adv_on)) {
        return;
    }

    LOG_INF("[ADV] Starting Cloned Advertising...");

    /* Create Dynamic Data */
    struct bt_data ad_data[2];
    int ad_len = 0;

    /* Flags */
    ad_data[ad_len].type = BT_DATA_FLAGS;
    ad_data[ad_len].data_len = 1;
    static uint8_t flags = (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR);
    ad_data[ad_len].data = &flags;
    ad_len++;

    /* Name */
    if (strlen(captured_name) > 0) {
        ad_data[ad_len].type = BT_DATA_NAME_COMPLETE;
        ad_data[ad_len].data_len = strlen(captured_name);
        ad_data[ad_len].data = (uint8_t *)captured_name;
        ad_len++;
    } else {
        // Fallback
        ad_data[ad_len].type = BT_DATA_NAME_COMPLETE;
        ad_data[ad_len].data_len = strlen("DE&N_RELAY");
        ad_data[ad_len].data = (uint8_t *)"DE&N_RELAY";
        ad_len++;
    }

    /* Scan Response Data (UUIDs) */
    struct bt_data sd_data[1];
    int sd_len = 0;

    if (captured_uuid_count > 0) {
        sd_data[sd_len].type = BT_DATA_UUID128_ALL;
        sd_data[sd_len].data_len = captured_uuid_count * 16;
        sd_data[sd_len].data = (uint8_t *)captured_uuids;
        sd_len++;
    } else {
        // Fallback
        sd_data[sd_len].type = BT_DATA_UUID128_ALL;
        sd_data[sd_len].data_len = 16;
        sd_data[sd_len].data = (uint8_t *)BT_UUID_BASE_SERVICE_VAL;
        sd_len++;
    }

    /* Use Extended Advertising */
    if (!adv) {
        struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
            BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
            BT_GAP_ADV_FAST_INT_MIN_2,
            BT_GAP_ADV_FAST_INT_MAX_2,
            NULL);

        err = bt_le_ext_adv_create(&param, NULL, &adv);
        if (err) {
            LOG_ERR("Failed to create advertiser (err %d)", err);
            return;
        }
    }

    err = bt_le_ext_adv_set_data(adv, ad_data, ad_len, sd_data, sd_len);
    if (err) {
        LOG_ERR("Failed to set adv data (err %d)", err);
        return;
    }

    err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) {
        LOG_ERR("Failed to start adv (err %d)", err);
        return;
    }

    atomic_set(&adv_on, 1);
    LOG_INF("[ADV] Started mimicking: %s", captured_name);
}

/* Helper to Forward Writes */
int ble_relay_send_write_to_dean(const struct bt_uuid *uuid, const void *data, uint16_t len)
{
    if (!central_conn) {
        LOG_WRN("[RELAY] No connection to DEAN");
        return -ENOTCONN;
    }

    uint16_t handle = 0;

    if (!bt_uuid_cmp(uuid, BT_UUID_CHRC_INFERENCE_RAWDATA)) {
        handle = h_remote_write_rawdata;
    } else if (!bt_uuid_cmp(uuid, BT_UUID_CHRC_FILE_TRANSFER)) {
        handle = h_remote_file_transfer;
    } else if (!bt_uuid_cmp(uuid, BT_UUID_CHRC_DEVICE_NAME)) {
        handle = h_remote_device_name;
    } else if (!bt_uuid_cmp(uuid, BT_UUID_CHRC_LOCATION)) {
        handle = h_remote_location;
    } else if (!bt_uuid_cmp(uuid, BT_UUID_CHRC_GRIDEYE_PREDICTION)) {
        handle = h_remote_grideye_prediction;
    }

    if (handle == 0) {
        LOG_WRN("[RELAY] Handle not found for UUID");
        return -EINVAL;
    }

    // Use Write Without Response for speed, or Write Response if needed.
    // Assuming Write Without Response for high throughput relaying unless confirmed otherwise.
    // However, for Config/Control, Write With Response is safer.
    // Let's use Write Request (default) to ensure it reaches DEAN.

    // Note: To use Write Without Response, we need to check properties.
    // For safety in this general function, we can try `bt_gatt_write_without_response`
    // and if it fails, fallback? Or just use `bt_gatt_write`.

    // Simpler approach: Just use Write Without Response as it's a relay.
    // If Ack is needed, SLIMHUB will handle app-level ack.

    int err = bt_gatt_write_without_response(central_conn, handle, data, len, false);
    if (err) {
        LOG_WRN("[RELAY] Write failed (err %d) handle=0x%04x", err, handle);
    } else {
        LOG_DBG("[RELAY] Forwarded %d bytes to handle 0x%04x", len, handle);
    }
    return err;
}
