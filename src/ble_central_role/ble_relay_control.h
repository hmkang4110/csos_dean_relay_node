#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#define MAX_RELAY_SESSIONS 5
#define RELAY_MAX_UUID16   6
#define RELAY_MAX_UUID128  2
#define RELAY_MAX_SERV_DATA_LEN 24
#define RELAY_MAX_MFG_DATA_LEN  24
#define RELAY_MAX_SUBS_PER_SESSION 24

#define BT_GAP_MAX_NAME_LEN 30

struct dean_adv_report {
    bt_addr_le_t addr;
    int8_t rssi;
    uint8_t adv_type;

    char name[BT_GAP_MAX_NAME_LEN];
    uint16_t uuid16[RELAY_MAX_UUID16];
    size_t uuid16_count;
    uint8_t uuid128[RELAY_MAX_UUID128][16];
    size_t uuid128_count;
    uint8_t uuid128_type;

    uint8_t service_data[RELAY_MAX_SERV_DATA_LEN];
    size_t service_data_len;
    uint8_t service_data_type;

    uint8_t mfg_data[RELAY_MAX_MFG_DATA_LEN];
    size_t mfg_data_len;
};

struct relay_session {
    uint8_t index;
    bool is_active;

    struct dean_adv_report adv_report;

    struct bt_conn *conn_dean;
    struct bt_conn *conn_slimhub;

    struct bt_le_ext_adv *adv;
    int adv_id;

    uint16_t handle_dean_inference_rawdata;
    uint16_t handle_dean_inference_seq_result;
    uint16_t handle_dean_inference_debugstr;

    struct bt_gatt_subscribe_params subs[RELAY_MAX_SUBS_PER_SESSION];
    size_t subs_count;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_discover_params ccc_discover_params;
    struct bt_gatt_subscribe_params *pending_sub;
    size_t ccc_next_idx;
};

extern struct relay_session g_relay_sessions[MAX_RELAY_SESSIONS];

int ble_relay_control_start(void);
void relay_route_write_to_dean(struct bt_conn *from_conn, const struct bt_gatt_attr *attr, const void *data, uint16_t length);
void relay_forward_notify_from_dean(struct bt_conn *conn, uint16_t handle, const void *data, uint16_t length);
