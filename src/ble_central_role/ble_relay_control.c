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
#include "ble.h"
#include "config_service.h"
#include "grideye_service.h"
#include "peripheral_service.h"
#include "env_service.h"
#include "sound_service.h"
#include "inference_service.h"
#include "ubinos_service.h"

#define MAX_SUBS 24

LOG_MODULE_REGISTER(central_scan, LOG_LEVEL_INF);

/* FUNCTION PRE-DEFINITIONS */
static void reset_work_handler(struct k_work *work);
static void adv_restart_work_handler(struct k_work *work);
static void scan_restart_work_handler(struct k_work *work);
static void initiate_timeout_work_handler(struct k_work *work);

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

static struct bt_gatt_discover_params discover_params;

K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(scan_restart_work, scan_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);
K_WORK_DELAYABLE_DEFINE(initiating_timeout_work, initiate_timeout_work_handler);

/* GLOBAL PARAMETER DEFINITIONS */
static uint32_t adv_backoff_ms = 200;
static uint32_t scan_backoff_ms = 200;
static uint32_t initiate_start_ms = 0;
#define BACKOFF_CAP 2000

static atomic_t adv_on;
static atomic_t scan_on;
static atomic_t initiating;

static struct bt_gatt_subscribe_params subs[MAX_SUBS];
static size_t subs_cnt;

struct adv_match_ctx
{
    bool name_match;
    char found_name[20];    // BT_GAP_MAX_NAME_LEN
};
static struct bt_conn *conn_connecting;

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

    err = bt_le_adv_start(BT_LE_ADV_CONN,
                          adv_data,
                          ARRAY_SIZE(adv_data),
                          scan_rsp_data,
                          ARRAY_SIZE(scan_rsp_data));
    if (err == -EALREADY) {
        LOG_INF("[ADV] adv already on");
        atomic_set(&adv_on, 1);
        adv_backoff_ms = 200;
        return;
    }

    if (err == -EBUSY) {
        LOG_WRN("[ADV] adv start busy, backoff %d ms", adv_backoff_ms);
        adv_backoff_ms = MIN(adv_backoff_ms * 2, BACKOFF_CAP);
        k_work_reschedule(&adv_restart_work, K_MSEC(adv_backoff_ms));
        return;
    }

    if (!err) {
        atomic_set(&adv_on, 1);
        adv_backoff_ms = 200;
        scan_start_safe(1000);

        err = hci_vs_write_adv_tx_power(20);
        if (err == 0) {
            int8_t eff;
            if (hci_vs_read_adv_tx_power(&eff) == 0) {
                LOG_INF("[HCI] ADV TX set=20 dBm, effective=%d dBm%s",
                        eff, (eff > 8) ? "  <-- FEM-updated" : "");
            } else {
                LOG_ERR("[HCI] READ adv TX failed");
            }
        } else {
            LOG_ERR("[HCI] WRITE adv TX(20) failed (%d)", err);
        }
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

    bool is_uuid_exist = false;
    struct bt_uuid adv_uuid;

    if (conn_connecting) {
        return;
    }

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
                            &conn_connecting);
    if (err) {
        LOG_WRN("[DEVICE FOUND] Create connection to %s failed (err %d)", addr_str, err);
        atomic_set(&initiating, 0);
        scan_start_safe(300);
        return;
    } else {
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
    if (!data) {
        LOG_INF("[NOTIFY] Unsubscribed from handle %u", params->value_handle);
        params->value_handle = 0;
        return BT_GATT_ITER_STOP;
    }

    const uint8_t *p = data;
    char buf[128];
    int off = 0;

    off += snprintk(buf + off, sizeof(buf) - off,
                    "[NOTIFY] handle=%u len=%u data=",
                    params->value_handle, length);

    for (uint16_t i = 0; i < length && off < (int)sizeof(buf) - 3; i++) {
        off += snprintk(buf + off, sizeof(buf) - off, "%02X ", p[i]);
    }

    LOG_INF("%s", buf);
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
            LOG_WRN("[CONNECTED] Failed to connect to %s (err %u)", addr, conn_err);
            if (conn) {
                bt_conn_unref(conn);
            }
            conn_connecting = NULL;
            atomic_set(&initiating, 0);
            scan_start_safe(300);
            return;
        } else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
            LOG_WRN("[CONNECTED] Failed to connect to central (err %u)", conn_err);
            adv_start_safe(300);
            return;
        }
    } else {
        /* connection success */
        if (info.role == BT_CONN_ROLE_CENTRAL) {
            if (conn == conn_connecting) {
                err = start_discovery(conn);
                if (err) {
                    LOG_WRN("[CONNECTED] start discovery error : %d", err);
                    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                } else {
                    LOG_INF("[CONNECTED] Connection established as central");
                }
            }

            conn_connecting = NULL;
            atomic_set(&initiating, 0);
            LOG_INF("[CONNECTED] New peripheral device connected : %s", addr);
        } else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
            LOG_INF("[CONNECTED] Connection established as peripheral : %s", addr);
            atomic_set(&adv_on, 0);
        }
    }

    LOG_INF("[CONNECTED] Connected: %s", addr);
    atomic_set(&initiating, 0);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("[DISCONNECTED] Disconnected from %s (reason %u), scan restart",
            addr, reason);

    if (conn) {
        bt_conn_unref(conn);
    }
    conn_connecting = NULL;
    atomic_set(&initiating, 0);
    scan_start_safe(300);
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
