#include "ble_relay_control.h"

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/atomic.h>

#include "relay_scan.h"
#include "relay_adv_clone.h"
#include "relay_mapping.h"
#include "relay_gatt_router.h"
#include "inference_service.h"

LOG_MODULE_REGISTER(central_scan, LOG_LEVEL_INF);

static atomic_t initiating;
static struct bt_conn *central_pending;
static struct relay_session *pending_session;

static void on_scan_match(const struct dean_adv_report *report);
static int start_discovery(struct relay_session *session);
static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr, struct bt_gatt_discover_params *params);
static uint8_t notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);

static void connected(struct bt_conn *conn, uint8_t err);
static void disconnected(struct bt_conn *conn, uint8_t reason);

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void on_scan_match(const struct dean_adv_report *report)
{
    struct relay_session *session;
    struct bt_conn *tmp_conn = NULL;
    int err;

    if (atomic_get(&initiating)) {
        return;
    }

    if (report->rssi < -90) {
        return;
    }

    session = relay_mapping_alloc(report);
    if (!session) {
        LOG_WRN("[SCAN] no free relay session");
        return;
    }

    atomic_set(&initiating, 1);
    relay_scan_stop(); /* keep controller free before connect */
    err = bt_conn_le_create(&report->addr,
                            BT_CONN_LE_CREATE_CONN,
                            BT_LE_CONN_PARAM_DEFAULT,
                            &tmp_conn);
    if (err) {
        LOG_WRN("[SCAN] create conn failed (%d)", err);
        relay_mapping_release(session);
        atomic_set(&initiating, 0);
        relay_scan_start(on_scan_match);
        return;
    }

    relay_scan_stop();
    central_pending = bt_conn_ref(tmp_conn);
    pending_session = session;
    bt_conn_unref(tmp_conn);
    LOG_INF("[SCAN] initiating connect to %s", report->name);
}

static struct relay_session *session_by_adv_id(uint8_t adv_id)
{
    for (int i = 0; i < MAX_RELAY_SESSIONS; i++) {
        if (g_relay_sessions[i].is_active &&
            g_relay_sessions[i].adv_id == adv_id) {
            return &g_relay_sessions[i];
        }
    }
    return NULL;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
    struct bt_conn_info info;
    char addr[BT_ADDR_LE_STR_LEN];
    struct relay_session *session = NULL;

    bt_conn_get_info(conn, &info);
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (conn_err) {
        LOG_WRN("[CONN] failed (%u) %s", conn_err, addr);

        if (info.role == BT_CONN_ROLE_CENTRAL && pending_session) {
            relay_mapping_release(pending_session);
            pending_session = NULL;
        }
        if (central_pending) {
            bt_conn_unref(central_pending);
            central_pending = NULL;
        }
        atomic_set(&initiating, 0);
        relay_scan_start(on_scan_match);
        return;
    }

    if (info.role == BT_CONN_ROLE_CENTRAL) {
        if (central_pending == conn && pending_session) {
            bt_conn_unref(central_pending);
            central_pending = NULL;
            session = pending_session;
            pending_session = NULL;
            session->conn_dean = bt_conn_ref(conn);
        } else {
            session = relay_mapping_by_dean_conn(conn);
            if (session && !session->conn_dean) {
                session->conn_dean = bt_conn_ref(conn);
            }
        }

        if (!session) {
            LOG_WRN("[CONN] central connection not mapped, dropping");
            bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
            return;
        }

        LOG_INF("[SESSION %d] connected to DEAN %s", session->index, addr);
        relay_adv_clone_start(session);
        start_discovery(session);
        atomic_set(&initiating, 0);
    } else {
        session = session_by_adv_id(info.id);
        if (!session || !session->conn_dean) {
            LOG_WRN("[CONN] hub connect but session missing, reject");
            bt_conn_disconnect(conn, BT_HCI_ERR_UNACCEPT_CONN_PARAM);
            return;
        }

        session->conn_slimhub = bt_conn_ref(conn);
        LOG_INF("[SESSION %d] SLIMHUB connected (%s)", session->index, addr);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    struct relay_session *session;
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    session = relay_mapping_by_dean_conn(conn);
    if (session) {
        LOG_INF("[SESSION %d] DEAN disconnected (%s, reason %u)", session->index, addr, reason);
        relay_adv_clone_stop(session);
        relay_mapping_release(session);
        atomic_set(&initiating, 0);
        relay_scan_start(on_scan_match);
        return;
    }

    session = relay_mapping_by_hub_conn(conn);
    if (session) {
        LOG_INF("[SESSION %d] SLIMHUB disconnected (%s, reason %u)", session->index, addr, reason);
        bt_conn_unref(session->conn_slimhub);
        session->conn_slimhub = NULL;
        relay_adv_clone_start(session);
        return;
    }

    LOG_INF("[DISCON] unmapped %s reason %u", addr, reason);
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    struct relay_session *session = relay_mapping_by_dean_conn(conn);
    const struct bt_gatt_chrc *chrc;
    struct bt_gatt_subscribe_params *sub;

    if (!session) {
        return BT_GATT_ITER_STOP;
    }

    if (!attr) {
        LOG_INF("[SESSION %d] discovery complete", session->index);
        memset(&session->discover_params, 0, sizeof(session->discover_params));
        relay_router_attach_dean_handles(session,
                                         session->handle_dean_inference_rawdata,
                                         session->handle_dean_inference_seq_result,
                                         session->handle_dean_inference_debugstr);
        return BT_GATT_ITER_STOP;
    }

    chrc = attr->user_data;
    if (params->type != BT_GATT_DISCOVER_CHARACTERISTIC) {
        return BT_GATT_ITER_CONTINUE;
    }

    if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
        if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_RAWDATA)) {
            session->handle_dean_inference_rawdata = chrc->value_handle;
        } else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT)) {
            session->handle_dean_inference_seq_result = chrc->value_handle;
        } else if (!bt_uuid_cmp(chrc->uuid, BT_UUID_CHRC_INFERENCE_DEBUG_STRING)) {
            session->handle_dean_inference_debugstr = chrc->value_handle;
        }

        if (session->subs_count < RELAY_MAX_SUBS_PER_SESSION) {
            sub = &session->subs[session->subs_count];
            memset(sub, 0, sizeof(*sub));
            /* With CONFIG_BT_GATT_AUTO_DISCOVER_CCC, set ccc_handle=0 to auto-find */
            sub->ccc_handle = 0;
            sub->value_handle = chrc->value_handle;
            sub->value = BT_GATT_CCC_NOTIFY;
            sub->notify = notify_cb;

            int serr = bt_gatt_subscribe(conn, sub);
            if (serr == 0 || serr == -EALREADY) {
                session->subs_count++;
                LOG_INF("[SESSION %d] subscribed val=0x%04x ccc=0x%04x", session->index, sub->value_handle, sub->ccc_handle);
            } else {
                LOG_WRN("[SESSION %d] subscribe failed val=0x%04x ccc=0x%04x err=%d",
                        session->index, sub->value_handle, sub->ccc_handle, serr);
            }
        }
    }

    return BT_GATT_ITER_CONTINUE;
}

static int start_discovery(struct relay_session *session)
{
    int err;

    memset(&session->discover_params, 0, sizeof(session->discover_params));
    session->handle_dean_inference_rawdata = 0;
    session->handle_dean_inference_seq_result = 0;
    session->handle_dean_inference_debugstr = 0;
    session->subs_count = 0;
    memset(session->subs, 0, sizeof(session->subs));

    session->discover_params.uuid = NULL;
    session->discover_params.func = discover_func;
    session->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    session->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    session->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    err = bt_gatt_discover(session->conn_dean, &session->discover_params);
    if (err) {
        LOG_WRN("[SESSION %d] discover failed (%d)", session->index, err);
    } else {
        LOG_INF("[SESSION %d] discover started", session->index);
    }
    return err;
}

static uint8_t notify_cb(struct bt_conn *conn,
                         struct bt_gatt_subscribe_params *params,
                         const void *data,
                         uint16_t length)
{
    if (!data) {
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    relay_forward_notify_from_dean(conn, params->value_handle, data, length);
    return BT_GATT_ITER_CONTINUE;
}

int ble_relay_control_start(void)
{
    int err;

    relay_router_init();

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("BLE init failed (err %d)", err);
        return err;
    }

    err = settings_load();
    if (err) {
        LOG_WRN("Settings load failed (err %d)", err);
    }

    LOG_INF("BLE relay starting (dual role)");
    relay_scan_start(on_scan_match);
    return 0;
}
