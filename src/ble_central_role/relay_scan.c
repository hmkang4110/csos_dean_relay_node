#include "relay_scan.h"

#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(relay_scan, LOG_LEVEL_INF);

static relay_scan_match_cb match_cb;

static const struct bt_le_scan_param scan_param = {
    .type = BT_LE_SCAN_TYPE_ACTIVE,
    .options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
    .interval = 0x00A0, /* 100 ms */
    .window = 0x0050,   /* 50 ms */
    .timeout = 0,
};

static bool adv_parse_collect(struct bt_data *data, void *user_data)
{
    struct dean_adv_report *report = user_data;

    switch (data->type) {
    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_NAME_SHORTENED: {
        size_t n = MIN((size_t)data->data_len, sizeof(report->name) - 1);
        memcpy(report->name, data->data, n);
        report->name[n] = '\0';
        break;
    }
    case BT_DATA_UUID16_SOME:
    case BT_DATA_UUID16_ALL: {
        size_t count = MIN((size_t)data->data_len / 2, (size_t)RELAY_MAX_UUID16);
        for (size_t i = 0; i < count; i++) {
            report->uuid16[i] = sys_get_le16(&data->data[i * 2]);
        }
        report->uuid16_count = count;
        break;
    }
    case BT_DATA_UUID128_SOME:
    case BT_DATA_UUID128_ALL: {
        size_t count = MIN((size_t)data->data_len / 16, (size_t)RELAY_MAX_UUID128);
        for (size_t i = 0; i < count; i++) {
            memcpy(report->uuid128[i], &data->data[i * 16], 16);
        }
        report->uuid128_count = count;
        report->uuid128_type = data->type;
        break;
    }
    case BT_DATA_SVC_DATA16:
    case BT_DATA_SVC_DATA32:
    case BT_DATA_SVC_DATA128: {
        size_t n = MIN((size_t)data->data_len, (size_t)RELAY_MAX_SERV_DATA_LEN);
        memcpy(report->service_data, data->data, n);
        report->service_data_len = n;
        report->service_data_type = data->type;
        break;
    }
    case BT_DATA_MANUFACTURER_DATA: {
        size_t n = MIN((size_t)data->data_len, (size_t)RELAY_MAX_MFG_DATA_LEN);
        memcpy(report->mfg_data, data->data, n);
        report->mfg_data_len = n;
        break;
    }
    default:
        break;
    }

    return true;
}

static bool is_dean_signature(const struct dean_adv_report *report)
{
    if (strstr(report->name, "DE&N")) {
        return true;
    }
    return false;
}

static void scan_device_found(const bt_addr_le_t *addr,
                              int8_t rssi,
                              uint8_t type,
                              struct net_buf_simple *ad)
{
    if (!match_cb || !ad || ad->len == 0) {
        return;
    }

    if (type != BT_GAP_ADV_TYPE_ADV_IND &&
        type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND &&
        type != BT_GAP_ADV_TYPE_EXT_ADV &&
        type != BT_GAP_ADV_TYPE_SCAN_RSP) {
        return;
    }

    struct dean_adv_report report = {0};
    bt_addr_le_copy(&report.addr, addr);
    report.rssi = rssi;
    report.adv_type = type;

    bt_data_parse(ad, adv_parse_collect, &report);

    if (!is_dean_signature(&report)) {
        return;
    }

    match_cb(&report);
}

int relay_scan_start(relay_scan_match_cb cb)
{
    int err;

    match_cb = cb;
    err = bt_le_scan_start(&scan_param, scan_device_found);
    if (err == -EALREADY) {
        LOG_INF("[SCAN] already running");
        return 0;
    }

    if (err) {
        LOG_ERR("[SCAN] start failed (%d)", err);
        return err;
    }

    LOG_INF("[SCAN] started (extended)");
    return 0;
}

void relay_scan_stop(void)
{
    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        LOG_WRN("[SCAN] stop failed (%d)", err);
    }
}
