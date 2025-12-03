#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/types.h>

#include <zephyr/ipc/ipc_service.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/sys/util.h>

// #include "inference.h"
// #include "inference_msgq.h"
#include "inference_service.h"
#include "ble_relay_control.h"

static bool inference_rawdata_notify_enabled;
static bool inference_seq_anal_result_notify_enabled;
static bool inference_debug_string_notify_enabled;

bool is_inference_notify_enabled(void)
{
    return inference_rawdata_notify_enabled;
}

bool is_inference_seq_anal_result_notify_enabled(void)
{
    return inference_seq_anal_result_notify_enabled;
}

bool is_inference_debug_string_notify_enabled(void)
{
    return inference_debug_string_notify_enabled;
}

static void ccc_cfg_inference_rawdata_changed(const struct bt_gatt_attr *attr,
                                uint16_t value)
{
    inference_rawdata_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void ccc_cfg_inference_seq_anal_result_changed(const struct bt_gatt_attr *attr,
                                uint16_t value)
{
    inference_seq_anal_result_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void ccc_cfg_inference_debug_string_changed(const struct bt_gatt_attr *attr,
                                uint16_t value)
{
    inference_debug_string_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

// delete later
static void unitspace_existence_estimation(const void *buf, uint16_t len)
{
    // printk("Unitspace Existence Estimation Write CB called, len: %d\n", len);
    // for (int i = 0; i < len; i++)
    // {
    //     printk(" %02X", ((uint8_t *)buf)[i]);
    // }
    // printk("\n");
}

static bt_gatt_attr_write_func_t unitspace_existence_estimation_write_cb(struct bt_conn *conn,
                                                const struct bt_gatt_attr *attr,
                                                const void *buf,
                                                uint16_t len,
                                                uint16_t offset,
                                                uint8_t flags)
{
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    relay_route_write_to_dean(conn, attr, buf, len);
    return len;
}

/** @brief inference data service declaration */
BT_GATT_SERVICE_DEFINE(
    inference_svr,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_INFERENCE_SERVICE),
    BT_GATT_CHARACTERISTIC( BT_UUID_CHRC_INFERENCE_RAWDATA,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                            NULL, unitspace_existence_estimation_write_cb, NULL),
    BT_GATT_CCC(ccc_cfg_inference_rawdata_changed,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
    BT_GATT_CHARACTERISTIC( BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            NULL, NULL, NULL),   
    BT_GATT_CCC(ccc_cfg_inference_seq_anal_result_changed,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), 
    BT_GATT_CHARACTERISTIC( BT_UUID_CHRC_INFERENCE_DEBUG_STRING,
                            BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                            BT_GATT_PERM_READ,
                            NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_inference_debug_string_changed,
                            BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
    );


int bt_inference_rawdata_send(uint8_t *packet_arr)
{
    int err = 0;

    if(!inference_rawdata_notify_enabled)
    {
        return -EACCES;
    }
    // return 
    
    err = bt_gatt_notify(NULL, &inference_svr.attrs[2],
                  packet_arr,
                  INFERENCE_RESULT_PACKET_SIZE);

    return err;
}

int bt_inference_seq_anal_result_send(char *result_char_arr, uint16_t result_len_uint16_t)
{
    int err = 0;
    if (!inference_seq_anal_result_notify_enabled)
    {
        return -EACCES;
    }

    err = bt_gatt_notify(NULL, &inference_svr.attrs[5],
                         result_char_arr,
                         result_len_uint16_t);
}

int bt_inference_debug_string_send(char *debug_string_arr, uint16_t debug_string_len_uint16_t)
{
    int err = 0;
    if (!inference_debug_string_notify_enabled)
    {
        return -EACCES;
    }

    err = bt_gatt_notify(NULL, &inference_svr.attrs[8],
                         debug_string_arr,
                         debug_string_len_uint16_t);
}
