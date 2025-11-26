#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>

#define MAX_RELAY_SESSIONS 5

struct relay_session {
    uint8_t index;
    bool is_active;

    struct bt_conn *conn_dean;
    struct bt_conn *conn_slimhub;

    struct bt_le_ext_adv *adv;
    bt_addr_le_t adv_addr;
    int adv_id;

    uint16_t handle_dean_inference_rawdata;
    uint16_t handle_dean_inference_debugstr;
};

extern struct relay_session g_relay_sessions[MAX_RELAY_SESSIONS];
// struct relay_session * get_session_by_hub_conn(struct bt_conn *conn);
// struct relay_session * get_session_by_dean_conn(struct bt_conn *conn);

int ble_relay_control_start(void);
void relay_route_write_to_dean(struct bt_conn *from_conn, uint16_t handle, const void *data, uint16_t length);