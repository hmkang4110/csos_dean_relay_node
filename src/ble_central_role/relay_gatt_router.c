#include "relay_gatt_router.h"

#include <string.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "relay_mapping.h"
#include "inference_service.h"

LOG_MODULE_REGISTER(relay_router, LOG_LEVEL_INF);

extern struct bt_gatt_service_static inference_svr;

void relay_router_init(void)
{
    relay_mapping_init();
}

void relay_router_attach_dean_handles(struct relay_session *session,
                                      uint16_t rawdata,
                                      uint16_t seq_result,
                                      uint16_t debug_str)
{
    if (!session) {
        return;
    }

    session->handle_dean_inference_rawdata = rawdata;
    session->handle_dean_inference_seq_result = seq_result;
    session->handle_dean_inference_debugstr = debug_str;
}

static uint16_t pick_remote_handle(const struct relay_session *session,
                                   const struct bt_gatt_attr *attr)
{
    if (attr == &inference_svr.attrs[2]) {
        return session->handle_dean_inference_rawdata;
    }
    if (attr == &inference_svr.attrs[5]) {
        return session->handle_dean_inference_seq_result;
    }
    if (attr == &inference_svr.attrs[8]) {
        return session->handle_dean_inference_debugstr;
    }

    return 0;
}

void relay_route_write_to_dean(struct bt_conn *from_conn,
                               const struct bt_gatt_attr *attr,
                               const void *data,
                               uint16_t length)
{
    struct relay_session *session = relay_mapping_by_hub_conn(from_conn);
    uint16_t target_handle;
    int err;

    if (!session || !session->conn_dean) {
        LOG_WRN("[ROUTE] write but session missing");
        return;
    }

    target_handle = pick_remote_handle(session, attr);
    if (!target_handle) {
        LOG_WRN("[ROUTE] write attr not mapped");
        return;
    }

    err = bt_gatt_write_without_response(session->conn_dean,
                                         target_handle,
                                         data,
                                         length,
                                         false);
    if (err) {
        LOG_WRN("[ROUTE] forward write failed (%d)", err);
    }
}

void relay_forward_notify_from_dean(struct bt_conn *conn,
                                    uint16_t handle,
                                    const void *data,
                                    uint16_t length)
{
    struct relay_session *session = relay_mapping_by_dean_conn(conn);
    const struct bt_gatt_attr *attr = NULL;
    int err;

    if (!session || !session->conn_slimhub) {
        return;
    }

    if (handle == session->handle_dean_inference_rawdata) {
        attr = &inference_svr.attrs[2];
    } else if (handle == session->handle_dean_inference_seq_result) {
        attr = &inference_svr.attrs[5];
    } else if (handle == session->handle_dean_inference_debugstr) {
        attr = &inference_svr.attrs[8];
    }

    if (!attr) {
        LOG_WRN("[ROUTE] notify handle 0x%04x not mapped", handle);
        return;
    }

    err = bt_gatt_notify(session->conn_slimhub, attr, data, length);
    if (err) {
        LOG_WRN("[ROUTE] notify forward failed (%d)", err);
    }
}
