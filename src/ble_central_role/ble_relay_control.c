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
#include "ble_relay_control.h"
#include "ui.h"
#include "config_service.h"

struct dean_link;

LOG_MODULE_REGISTER(central_scan, LOG_LEVEL_INF);

/* FUNCTION PRE-DEFINITIONS */
static void reset_work_handler(struct k_work *work);
static void adv_restart_work_handler(struct k_work *work);
static void scan_restart_work_handler(struct k_work *work);
static void initiate_timeout_work_handler(struct k_work *work);
static void scan_idle_timeout_work_handler(struct k_work *work);
static void adv_enable_work_handler(struct k_work *work);

static void adv_start_safe(int delay_ms);
static void adv_stop_safe(void);
static void scan_start_safe(int delay_ms);
static void scan_stop_safe(void);
static void scan_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad);
static int start_discovery_link(struct dean_link *link);
static struct dean_link *find_link(struct bt_conn *conn);
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params);
static bool ad_parse_cb (struct bt_data * data, void *user_data);
static int hci_vs_write_adv_tx_power(int8_t tx_dbm);
static int hci_vs_read_adv_tx_power(int8_t *out_dbm);
static uint8_t generic_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
static void mtu_exchanged_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params);
static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);
static void forward_write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params);

K_WORK_DELAYABLE_DEFINE(adv_restart_work, adv_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(scan_restart_work, scan_restart_work_handler);
K_WORK_DELAYABLE_DEFINE(reset_work, reset_work_handler);
K_WORK_DELAYABLE_DEFINE(initiating_timeout_work, initiate_timeout_work_handler);
K_WORK_DELAYABLE_DEFINE(scan_idle_timeout_work, scan_idle_timeout_work_handler);
K_WORK_DELAYABLE_DEFINE(adv_enable_work, adv_enable_work_handler);

/* GLOBAL PARAMETER DEFINITIONS */
static uint32_t adv_backoff_ms = 200;
static uint32_t scan_backoff_ms = 200;
static uint32_t initiate_start_ms = 0;
#define BACKOFF_CAP 2000

static atomic_t adv_on;
static atomic_t scan_on;
static atomic_t initiating;

enum relay_phase {
	RELAY_PHASE_SCAN_DEAN = 0,
	RELAY_PHASE_ADV_SLIMHUB,
};

static enum relay_phase relay_phase = RELAY_PHASE_SCAN_DEAN;

/* If no DEAN progress (match/connect) is seen for this long, assume there are no more scannable nodes. */
#define SCAN_IDLE_TIMEOUT_MS 8000
static uint32_t last_dean_activity_ms;

struct adv_match_ctx
{
    bool name_match;
    char found_name[20];    // BT_GAP_MAX_NAME_LEN
};
static struct bt_conn *central_pending;
static struct bt_conn *peripheral_conn;

#define ROUTING_TABLE_SIZE 8
#define DEAN_MAC_LEN       6
#define MAX_DEAN_CONN      4
#define MAX_SUBS_PER_CONN  16

enum relay_char_type
{
    RELAY_CHAR_RAWDATA = 0,
    RELAY_CHAR_SEQ_RESULT,
    RELAY_CHAR_DEBUG_STRING,
    RELAY_CHAR_LOCATION,
};

struct relay_route_entry
{
    bool in_use;
    uint8_t mac[DEAN_MAC_LEN];
    struct bt_conn *conn;
    uint16_t h_rawdata;
    uint16_t h_seq_result;
    uint16_t h_debug_string;
    uint16_t h_location;
};

static struct relay_route_entry routing_table[ROUTING_TABLE_SIZE];

struct dean_link
{
    struct bt_conn *conn;
    struct bt_gatt_exchange_params mtu_params;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params subs[MAX_SUBS_PER_CONN];
    struct bt_gatt_write_params write_params;
    uint8_t write_buf[64];
    bool write_in_progress;
};

static struct dean_link dean_links[MAX_DEAN_CONN];

static void enter_scan_dean_phase(void)
{
	relay_phase = RELAY_PHASE_SCAN_DEAN;
	k_work_cancel_delayable(&adv_enable_work);
	k_work_cancel_delayable(&adv_restart_work);
	adv_stop_safe();
	scan_start_safe(0);
	last_dean_activity_ms = k_uptime_get_32();
	k_work_reschedule(&scan_idle_timeout_work, K_MSEC(SCAN_IDLE_TIMEOUT_MS));
}

static void enter_adv_slimhub_phase(int delay_ms)
{
	relay_phase = RELAY_PHASE_ADV_SLIMHUB;
	scan_stop_safe();
	k_work_cancel_delayable(&scan_idle_timeout_work);
	adv_start_safe(delay_ms);
}

static inline bool mac_equal(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, DEAN_MAC_LEN) == 0;
}

static inline void mac_from_addr(const bt_addr_le_t *addr, uint8_t out[DEAN_MAC_LEN])
{
    /* Convert controller-stored (LSB-first) addr->a.val into big-endian order */
    for (size_t i = 0; i < DEAN_MAC_LEN; i++) {
        out[i] = addr->a.val[DEAN_MAC_LEN - 1 - i];
    }
}

static const char *mac_to_str(const uint8_t mac[DEAN_MAC_LEN], char *buf, size_t len)
{
    if (!buf || len < MAC_ADDR_STR_LEN + 1) {
        return NULL;
    }

    snprintk(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return buf;
}

static struct relay_route_entry *routing_find_by_mac(const uint8_t mac[DEAN_MAC_LEN])
{
    for (size_t i = 0; i < ARRAY_SIZE(routing_table); i++) {
        if (routing_table[i].in_use && mac_equal(routing_table[i].mac, mac)) {
            return &routing_table[i];
        }
    }
    return NULL;
}

static struct relay_route_entry *routing_find_by_conn(struct bt_conn *conn)
{
    if (!conn) {
        return NULL;
    }

    for (size_t i = 0; i < ARRAY_SIZE(routing_table); i++) {
        if (routing_table[i].in_use && routing_table[i].conn == conn) {
            return &routing_table[i];
        }
    }
    return NULL;
}

static struct relay_route_entry *routing_alloc(const uint8_t mac[DEAN_MAC_LEN])
{
    for (size_t i = 0; i < ARRAY_SIZE(routing_table); i++) {
        if (!routing_table[i].in_use) {
            struct relay_route_entry *entry = &routing_table[i];
            memset(entry, 0, sizeof(*entry));
            entry->in_use = true;
            memcpy(entry->mac, mac, DEAN_MAC_LEN);
            return entry;
        }
    }
    return NULL;
}

static struct relay_route_entry *routing_touch(const uint8_t mac[DEAN_MAC_LEN])
{
    struct relay_route_entry *entry = routing_find_by_mac(mac);
    if (entry) {
        return entry;
    }

    entry = routing_alloc(mac);
    if (!entry) {
        char mac_buf[MAC_ADDR_STR_LEN + 1];
        mac_to_str(mac, mac_buf, sizeof(mac_buf));
        LOG_WRN("[ROUTE] table full, drop MAC %s", mac_buf);
        return NULL;
    }

    char mac_buf[MAC_ADDR_STR_LEN + 1];
    mac_to_str(mac, mac_buf, sizeof(mac_buf));
    LOG_INF("[ROUTE] added MAC %s to routing table", mac_buf);
    return entry;
}

static void routing_bind_conn(struct relay_route_entry *entry, struct bt_conn *conn)
{
    if (!entry) {
        return;
    }
    entry->conn = conn;
}

static void routing_track_connection(struct bt_conn *conn)
{
    const bt_addr_le_t *dst = bt_conn_get_dst(conn);
    if (!dst) {
        return;
    }

    uint8_t mac[DEAN_MAC_LEN];
    mac_from_addr(dst, mac);

    struct relay_route_entry *entry = routing_touch(mac);
    if (entry) {
        routing_bind_conn(entry, conn);
    }
}

static void routing_update_handle(struct bt_conn *conn,
                                  enum relay_char_type type,
                                  uint16_t handle)
{
    struct relay_route_entry *entry = routing_find_by_conn(conn);

    if (!entry) {
        const bt_addr_le_t *dst = bt_conn_get_dst(conn);
        if (dst) {
            uint8_t mac[DEAN_MAC_LEN];
            mac_from_addr(dst, mac);
            entry = routing_touch(mac);
            if (entry && !entry->conn) {
                routing_bind_conn(entry, conn);
            }
        }
    }

    if (!entry) {
        LOG_WRN("[ROUTE] skip handle update: conn not tracked (0x%04x)", handle);
        return;
    }

    uint16_t old = 0;
    switch (type) {
    case RELAY_CHAR_RAWDATA:
        old = entry->h_rawdata;
        entry->h_rawdata = handle;
        break;
    case RELAY_CHAR_SEQ_RESULT:
        old = entry->h_seq_result;
        entry->h_seq_result = handle;
        break;
    case RELAY_CHAR_DEBUG_STRING:
        old = entry->h_debug_string;
        entry->h_debug_string = handle;
        break;
    case RELAY_CHAR_LOCATION:
        old = entry->h_location;
        entry->h_location = handle;
        break;
    default:
        break;
    }

    if (old != handle) {
        char mac_buf[MAC_ADDR_STR_LEN + 1];
        mac_to_str(entry->mac, mac_buf, sizeof(mac_buf));
        const char *type_str = (type == RELAY_CHAR_RAWDATA) ? "RAWDATA" :
                               (type == RELAY_CHAR_SEQ_RESULT) ? "SEQ_RESULT" :
                               (type == RELAY_CHAR_DEBUG_STRING) ? "DEBUG_STR" :
                               (type == RELAY_CHAR_LOCATION) ? "LOCATION" : "UNKNOWN";
        LOG_INF("[ROUTE] %s handle=0x%04x (mac=%s)", type_str, handle, mac_buf);
    }
}

static void routing_remove_conn(struct bt_conn *conn)
{
    if (!conn) {
        return;
    }

    for (size_t i = 0; i < ARRAY_SIZE(routing_table); i++) {
        if (routing_table[i].in_use && routing_table[i].conn == conn) {
            char mac_buf[MAC_ADDR_STR_LEN + 1];
            mac_to_str(routing_table[i].mac, mac_buf, sizeof(mac_buf));
            LOG_INF("[ROUTE] removing entry for %s", mac_buf);
            memset(&routing_table[i], 0, sizeof(routing_table[i]));
        }
    }
}

int relay_forward_rawdata_to_dean(const uint8_t mac[DEAN_MAC_LEN],
                                  const uint8_t *payload,
                                  uint16_t len)
{
    struct relay_route_entry *entry = routing_find_by_mac(mac);
    struct dean_link *link = NULL;
    if (entry) {
        link = find_link(entry->conn);
    }

    if (!entry || !entry->conn || !entry->h_rawdata || !link) {
        return -ENODEV;
    }

    if (link->write_in_progress) {
        return -EBUSY;
    }

    if (len > sizeof(link->write_buf)) {
        return -EMSGSIZE;
    }

    memcpy(link->write_buf, payload, len);
    link->write_params.func = forward_write_cb;
    link->write_params.handle = entry->h_rawdata;
    link->write_params.offset = 0;
    link->write_params.data = link->write_buf;
    link->write_params.length = len;
    link->write_in_progress = true;

    int err = bt_gatt_write(entry->conn, &link->write_params);
    if (err) {
        char mac_buf[MAC_ADDR_STR_LEN + 1];
        mac_to_str(mac, mac_buf, sizeof(mac_buf));
        LOG_WRN("[ROUTE] forward rawdata to %s failed err=%d", mac_buf, err);
        link->write_in_progress = false;
    } else {
        ui_status_relay_activity();
    }
    return err;
}

int relay_forward_location_to_dean(const uint8_t mac[DEAN_MAC_LEN],
                                   const uint8_t *payload,
                                   uint16_t len)
{
    struct relay_route_entry *entry = routing_find_by_mac(mac);
    struct dean_link *link = NULL;
    if (entry) {
        link = find_link(entry->conn);
    }

    if (!entry || !entry->conn || !entry->h_location || !link) {
        return -ENODEV;
    }

    if (link->write_in_progress) {
        return -EBUSY;
    }

    if (len > sizeof(link->write_buf)) {
        return -EMSGSIZE;
    }

    memcpy(link->write_buf, payload, len);
    link->write_params.func = forward_write_cb;
    link->write_params.handle = entry->h_location;
    link->write_params.offset = 0;
    link->write_params.data = link->write_buf;
    link->write_params.length = len;
    link->write_in_progress = true;

    int err = bt_gatt_write(entry->conn, &link->write_params);
    if (err) {
        char mac_buf[MAC_ADDR_STR_LEN + 1];
        mac_to_str(mac, mac_buf, sizeof(mac_buf));
        LOG_WRN("[ROUTE] forward location to %s failed err=%d", mac_buf, err);
        link->write_in_progress = false;
    }
    return err;
}

static struct dean_link *find_link(struct bt_conn *conn)
{
    if (!conn) {
        return NULL;
    }
    for (size_t i = 0; i < ARRAY_SIZE(dean_links); i++) {
        if (dean_links[i].conn == conn) {
            return &dean_links[i];
        }
    }
    return NULL;
}

static struct dean_link *alloc_link(struct bt_conn *conn, bool already_refed)
{
    for (size_t i = 0; i < ARRAY_SIZE(dean_links); i++) {
        if (!dean_links[i].conn) {
            struct dean_link *link = &dean_links[i];
            memset(link, 0, sizeof(*link));
            link->conn = already_refed ? conn : bt_conn_ref(conn);
            return link;
        }
    }
    return NULL;
}

static size_t dean_active_count(void)
{
    size_t cnt = 0;
    for (size_t i = 0; i < ARRAY_SIZE(dean_links); i++) {
        if (dean_links[i].conn) {
            cnt++;
        }
    }
    return cnt;
}

static void free_link(struct dean_link *link)
{
    if (!link) {
        return;
    }
    if (link->conn) {
        bt_conn_unref(link->conn);
    }
    memset(link, 0, sizeof(*link));
}

static bool dean_has_space(void)
{
    return dean_active_count() < ARRAY_SIZE(dean_links);
}

static void forward_write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
    struct dean_link *link = find_link(conn);
    if (link) {
        link->write_in_progress = false;
    }

    if (err) {
        LOG_WRN("[ROUTE] downstream write rsp err=%u", err);
    }
}

static struct dean_link *find_link_by_sub(struct bt_gatt_subscribe_params *params,
                                          struct bt_gatt_subscribe_params **out_sub)
{
    if (out_sub) {
        *out_sub = NULL;
    }
    if (!params) {
        return NULL;
    }
    for (size_t i = 0; i < ARRAY_SIZE(dean_links); i++) {
        for (size_t j = 0; j < MAX_SUBS_PER_CONN; j++) {
            if (&dean_links[i].subs[j] == params) {
                if (out_sub) {
                    *out_sub = &dean_links[i].subs[j];
                }
                return &dean_links[i];
            }
        }
    }
    return NULL;
}


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

    if (relay_phase != RELAY_PHASE_SCAN_DEAN) {
        return;
    }

    if (atomic_get(&initiating) == 1 || central_pending) {
        return;
    }

    if (!dean_has_space()) {
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

    if (relay_phase != RELAY_PHASE_ADV_SLIMHUB) {
        return;
    }

    /* Only advertise to accept SLIMHUB; do not advertise while already connected as peripheral. */
    if (peripheral_conn) {
        atomic_set(&adv_on, 0);
        return;
    }

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
        if (central_pending)
        {
            bt_conn_unref(central_pending);
            central_pending = NULL;
        }
        scan_start_safe(300);
    }
}

static void scan_idle_timeout_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (relay_phase != RELAY_PHASE_SCAN_DEAN) {
		return;
	}

	/* Keep scanning until we reach the maximum number of DEAN links. */
	if (!dean_has_space()) {
		enter_adv_slimhub_phase(0);
		return;
	}

	if (atomic_get(&initiating) == 1 || central_pending) {
		k_work_reschedule(&scan_idle_timeout_work, K_MSEC(SCAN_IDLE_TIMEOUT_MS));
		return;
	}

	/* Ensure scanning stays active while we still have DEAN slots. */
	scan_start_safe(0);

	k_work_reschedule(&scan_idle_timeout_work, K_MSEC(SCAN_IDLE_TIMEOUT_MS));
}

static void adv_enable_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	enter_adv_slimhub_phase(0);
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

    if (relay_phase != RELAY_PHASE_SCAN_DEAN) {
        return;
    }

    if (central_pending || !dean_has_space()) {
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

    last_dean_activity_ms = k_uptime_get_32();
    k_work_reschedule(&scan_idle_timeout_work, K_MSEC(SCAN_IDLE_TIMEOUT_MS));

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

        char target_peripheral_name[20] = "DE&N_TERMINAL";

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
    struct dean_link *link = CONTAINER_OF(params, struct dean_link, discover_params);

    /* 1) 탐색 종료 조건 */
    if (!attr) {
        LOG_INF("[DISCOVER] type %u complete", params->type);
        memset(params, 0, sizeof(*params));   /* 이 discover 작업은 끝 */
        /* resume scanning only after this link's discovery/subscriptions are done */
        if (relay_phase == RELAY_PHASE_SCAN_DEAN && dean_has_space()) {
            atomic_set(&scan_on, 0);
            scan_start_safe(300);
        }
        return BT_GATT_ITER_STOP;
    }

    /* 2) 우리는 CHARACTERISTIC 탐색만 사용 중 */
    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chrc = attr->user_data;
        uint16_t decl_handle  = attr->handle;          /* Characteristic Declaration */
        uint16_t value_handle = chrc->value_handle;    /* Characteristic Value */
        const char *inf_name = NULL;

        /* Track writable config characteristics even if they are not notifiable. */
        if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_LOCATION)) {
            routing_update_handle(conn, RELAY_CHAR_LOCATION, value_handle);
        }

        /* LOG_DBG("[DISCOVER] char decl=0x%04x val=0x%04x props=0x%02x",
                   decl_handle, value_handle, chrc->properties); */

        /* 2-1) Notify 지원하는 Characteristic 인가? */
        if (chrc->properties & BT_GATT_CHRC_NOTIFY) {

            if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_RAWDATA))
            {
                inf_name = "INFERENCE_RAWDATA";
                routing_update_handle(conn, RELAY_CHAR_RAWDATA, value_handle);
            }
            else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT))
            {
                inf_name = "INFERENCE_SEQ_RESULT";
                routing_update_handle(conn, RELAY_CHAR_SEQ_RESULT, value_handle);
            }
            else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_DEBUG_STRING))
            {
                inf_name = "INFERENCE_DEBUG_STR";
                routing_update_handle(conn, RELAY_CHAR_DEBUG_STRING, value_handle);
            }

            if (!link) {
                LOG_WRN("[DISCOVER] link missing for subscription");
                return BT_GATT_ITER_CONTINUE;
            }

            struct bt_gatt_subscribe_params *sub = NULL;
            for (size_t i = 0; i < MAX_SUBS_PER_CONN; i++) {
                if (link->subs[i].value_handle == 0) {
                    sub = &link->subs[i];
                    break;
                }
            }

            if (!sub) {
                if (inf_name) {
                    LOG_WRN("[DISCOVER] %s subscribe skipped: table full (val=0x%04x)",
                            inf_name, value_handle);
                } else {
                    LOG_WRN("[DISCOVER] subscribe table full, skip");
                }
                return BT_GATT_ITER_CONTINUE;
            }

            memset(sub, 0, sizeof(*sub));

            /* 단순 가정: CCCD = value_handle + 1 */
            sub->ccc_handle   = (uint16_t)(value_handle + 1);
            sub->value_handle = value_handle;
            sub->value        = BT_GATT_CCC_NOTIFY;
            sub->notify       = generic_notify_cb;

            if (inf_name) {
                LOG_INF("[SUB] %s subscribe attempt val=0x%04x ccc=0x%04x decl=0x%04x",
                        inf_name, sub->value_handle, sub->ccc_handle, decl_handle);
            }

            int err = bt_gatt_subscribe(conn, sub);
            if (err && err != -EALREADY) {
                LOG_WRN("[DISCOVER] subscribe failed: val=0x%04x ccc=0x%04x err=%d",
                        sub->value_handle, sub->ccc_handle, err);
                memset(sub, 0, sizeof(*sub));
            } else if (inf_name) {
                LOG_INF("[SUB] %s subscribe ok (err=%d) val=0x%04x ccc=0x%04x",
                        inf_name, err, sub->value_handle, sub->ccc_handle);
            } else {
                /* subscription succeeded */
            }
        }

        return BT_GATT_ITER_CONTINUE;
    }

    /* 지금은 다른 type 을 쓰지 않지만, 확장 대비 */
    LOG_DBG("[DISCOVER] unsupported discover type=%u at handle=0x%04x",
            params->type, attr->handle);
    return BT_GATT_ITER_CONTINUE;
}

static int start_discovery_link(struct dean_link *link)
{
    if (!link || !link->conn) {
        return -EINVAL;
    }

    int err;
    struct bt_gatt_discover_params *discover_params = &link->discover_params;

    memset(discover_params, 0, sizeof(*discover_params));

    /* 서비스 UUID를 모른다는 가정 → ATT 전체 범위에서
     * 모든 Characteristic 을 한 번 훑는다.
     */
    discover_params->uuid         = NULL; /* 모든 캐릭터리스틱 */
    discover_params->func         = discover_func;
    discover_params->start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_params->end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_params->type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    err = bt_gatt_discover(link->conn, discover_params);
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

    if (length < DEAN_MAC_LEN) {
        LOG_WRN("[NOTIFY] payload too short (%u)", length);
        return BT_GATT_ITER_CONTINUE;
    }

    const uint8_t *mac = data;
    struct relay_route_entry *entry = routing_touch(mac);
    struct relay_route_entry *conn_entry = routing_find_by_conn(conn);

    if (!entry && conn_entry) {
        entry = conn_entry;
    } else if (entry && conn_entry && entry != conn_entry) {
        if (!entry->h_rawdata && conn_entry->h_rawdata) {
            entry->h_rawdata = conn_entry->h_rawdata;
        }
        if (!entry->h_seq_result && conn_entry->h_seq_result) {
            entry->h_seq_result = conn_entry->h_seq_result;
        }
        if (!entry->h_debug_string && conn_entry->h_debug_string) {
            entry->h_debug_string = conn_entry->h_debug_string;
        }
        if (!entry->h_location && conn_entry->h_location) {
            entry->h_location = conn_entry->h_location;
        }
    }

    if (entry && !entry->conn) {
        routing_bind_conn(entry, conn);
    }


    uint16_t handle = params->value_handle;
    if (entry && entry->h_rawdata && handle == entry->h_rawdata)
    {
        if (!is_inference_notify_enabled()) {
            return BT_GATT_ITER_CONTINUE;
        }
        err = bt_inference_rawdata_send((uint8_t *)data, length);
        if (err && err != -EACCES)
        {
            LOG_WRN("[RELAY] INFERENCE_RAWDATA send failed (err %d)", err);
        } else if (err == 0) {
            ui_status_relay_activity();
        }
    }
    else if (entry && entry->h_seq_result && handle == entry->h_seq_result)
    {
        if (!is_inference_seq_anal_result_notify_enabled()) {
            return BT_GATT_ITER_CONTINUE;
        }
        int err = bt_inference_seq_anal_result_send((char *)data, length);
        if (err && err != -EACCES)
        {
            LOG_WRN("[RELAY] INFERENCE_SEQ_ANAL_RESULT send failed (err %d)", err);
        } else if (err == 0) {
            ui_status_relay_activity();
        }
    }
    else if (entry && entry->h_debug_string && handle == entry->h_debug_string)
    {
        LOG_INF("[NOTIFY] INFERENCE_DEBUG_STR from conn handle=0x%04x len=%u", handle, length);
        if (!is_inference_debug_string_notify_enabled()) {
            LOG_INF("[NOTIFY] drop INFERENCE_DEBUG_STR forward: SLIMHUB CCC disabled");
            return BT_GATT_ITER_CONTINUE;
        }
        err = bt_inference_debug_string_send((uint8_t *)data, length);
        if (err && err != -EACCES)
        {
            LOG_WRN("[RELAY] INFERENCE_DEBUG_STRING send failed (err %d)", err);
        } else if (err == 0) {
            ui_status_relay_activity();
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

static void mtu_exchanged_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (!conn || !params) {
        return;
    }

    /* Start characteristic discovery after MTU exchange completes (success or failure). */
    struct dean_link *link = CONTAINER_OF(params, struct dean_link, mtu_params);
    if (link && link->conn == conn) {
        int d_err = start_discovery_link(link);
        if (d_err) {
            LOG_WRN("[MTU] start discovery error after MTU exchange: %d", d_err);
            bt_conn_disconnect(link->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        }
    }
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
            if (relay_phase == RELAY_PHASE_SCAN_DEAN) {
                scan_start_safe(300);
            }
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

            struct dean_link *link = NULL;
            if (central_pending == conn) {
                link = alloc_link(conn, true);
                central_pending = NULL;
            } else {
                link = alloc_link(conn, false);
            }

            if (!link) {
                LOG_WRN("[CONNECTED] No free central slot for %s", addr);
                bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                atomic_set(&initiating, 0);
                return;
            }

            routing_track_connection(link->conn);

            memset(&link->mtu_params, 0, sizeof(link->mtu_params));
            link->mtu_params.func = mtu_exchanged_cb;
            err = bt_gatt_exchange_mtu(link->conn, &link->mtu_params);
            if (err) {
                err = start_discovery_link(link);
                if (err) {
                    LOG_WRN("[CONNECTED] start discovery error : %d", err);
                    bt_conn_disconnect(link->conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
                }
            }

            ui_status_dean_connected();
            LOG_INF("[CONNECTED] Connection established as CENTRAL to peripheral %s", addr);

            atomic_set(&initiating, 0);
            LOG_INF("[CONNECTED] New peripheral device connected : %s", addr);

            /* Reset scan idle timer based on actual connection progress (not just scan matches). */
            last_dean_activity_ms = k_uptime_get_32();
            if (relay_phase == RELAY_PHASE_SCAN_DEAN) {
                k_work_reschedule(&scan_idle_timeout_work, K_MSEC(SCAN_IDLE_TIMEOUT_MS));
            }

	            /* If we've filled all DEAN slots, switch to advertising immediately. */
	            if (relay_phase == RELAY_PHASE_SCAN_DEAN && !dean_has_space()) {
	                enter_adv_slimhub_phase(0);
	            }
	        }
        else if (info.role == BT_CONN_ROLE_PERIPHERAL) {
            /* relay node 가 PERIPHERAL 로서 SLIMHUB 에 붙은 상황 */

            if (!peripheral_conn) {
                peripheral_conn = bt_conn_ref(conn);
            }

            ui_status_slimhub_connected();
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
        /* 여기서 conn 은 Zephyr 스택이 관리하는 포인터이므로 우리가 unref 하지 않음 */
        return;
    }

    bt_addr_le_to_str(info.le.dst, addr, sizeof(addr));

    if (info.role == BT_CONN_ROLE_PERIPHERAL) {
        /* relay node 가 PERIPHERAL 로서 SLIMHUB 에 붙어 있던 연결이 끊어진 경우 */
        LOG_INF("[DISCONNECTED] Central %s disconnected (reason %u) -> restart advertising",
                addr, reason);

        routing_remove_conn(conn);

        if (peripheral_conn == conn) {
            bt_conn_unref(peripheral_conn);
            peripheral_conn = NULL;
        }

        /* 필요하면 inference_svr 의 notify enable 플래그들 초기화 (옵션) */

        ui_status_slimhub_disconnected();
        atomic_set(&adv_on, 0);
        adv_start_safe(300);
    }
    else if (info.role == BT_CONN_ROLE_CENTRAL) {
        /* relay node 가 CENTRAL 로서 DEAN node 에 붙어 있던 연결이 끊어진 경우 */
        LOG_INF("[DISCONNECTED] Peripheral %s disconnected (reason %u) -> restart scanning",
                addr, reason);

        routing_remove_conn(conn);

        struct dean_link *link = find_link(conn);
        if (link) {
            LOG_INF("[DISCONNECTED] clearing subscriptions and freeing link for %s", addr);
            memset(link->subs, 0, sizeof(link->subs));
            free_link(link);
        }
        if (central_pending == conn) {
            bt_conn_unref(central_pending);
            central_pending = NULL;
        }

        atomic_set(&initiating, 0);

        ui_status_dean_disconnected();
        /* Losing a DEAN link returns us to the "scan DEAN first" phase. */
        enter_scan_dean_phase();
    } else {
        LOG_INF("[DISCONNECTED] Disconnected from %s (reason %u), unknown role=%d",
                addr, reason, info.role);
    }

    /* ⚠️ 여기서 bt_conn_unref(conn)을 호출하지 않는다!
     * 우리가 ref를 잡은 포인터에 대해서만 위에서 unref 했으므로,
     * conn 포인터는 Zephyr 스택이 알아서 정리한다.
     */
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

    enter_scan_dean_phase();

    return err;
}
