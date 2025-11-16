#include "ble.h"
#include <zephyr/bluetooth/gatt.h>


/** @brief PAAR Device's scan response packet index number*/
#define PAAR_DEVICE_IDX	                                10
#define BT_UUID_PAAR_SERVICE_UUID_FIRST                 0x6e402650
#define BT_UUID_PAAR_SERVICE_UUID_SECOND                0xb5a3
#define BT_UUID_PAAR_SERVICE_UUID_THIRD                 0xf393
#define BT_UUID_PAAR_SERVICE_UUID_FOURTH                0xe0a9
#define BT_UUID_PAAR_SERVICE_UUID_FIFTH                 0xe50e24dcca9e
#define BT_UUID_PAAR_CHRC_TX_UUID_FIRST                 0x6e407f01
#define BT_UUID_PAAR_CHRC_TX_UUID_SECOND                0xb5a3
#define BT_UUID_PAAR_CHRC_TX_UUID_THIRD                 0xf393
#define BT_UUID_PAAR_CHRC_TX_UUID_FOURTH                0xe0a9
#define BT_UUID_PAAR_CHRC_TX_UUID_FIFTH                 0xe50e24dcca9e
#define BT_UUID_PAAR_CHRC_RX_UUID_FIRST                 0x6e407f02
#define BT_UUID_PAAR_CHRC_RX_UUID_SECOND                0xb5a3
#define BT_UUID_PAAR_CHRC_RX_UUID_THIRD                 0xf393
#define BT_UUID_PAAR_CHRC_RX_UUID_FOURTH                0xe0a9
#define BT_UUID_PAAR_CHRC_RX_UUID_FIFTH                 0xe50e24dcca9e

#define BT_UUID_UBINOS_SERVICE_UUID_FIRST	0x6e400001
#define BT_UUID_UBINOS_CHRC_RX_UUID_FIRST	0x6e400002
#define BT_UUID_UBINOS_CHRC_TX_UUID_FIRST	0x6e400003

/** @brief PAAR device's uuid definitions */
#define BT_UUID_PAAR_SERVICE_VAL                         \
    BT_UUID_128_ENCODE(BT_UUID_PAAR_SERVICE_UUID_FIRST,  \
                       BT_UUID_PAAR_SERVICE_UUID_SECOND, \
                       BT_UUID_PAAR_SERVICE_UUID_THIRD,  \
                       BT_UUID_PAAR_SERVICE_UUID_FOURTH, \
                       BT_UUID_PAAR_SERVICE_UUID_FIFTH)
#define BT_UUID_PAAR_SERVICE BT_UUID_DECLARE_128(BT_UUID_PAAR_SERVICE_VAL)
#define BT_UUID_PAAR_CHRC_TX_VAL                         \
    BT_UUID_128_ENCODE(BT_UUID_PAAR_CHRC_TX_UUID_FIRST,  \
                       BT_UUID_PAAR_CHRC_TX_UUID_SECOND, \
                       BT_UUID_PAAR_CHRC_TX_UUID_THIRD,  \
                       BT_UUID_PAAR_CHRC_TX_UUID_FOURTH, \
                       BT_UUID_PAAR_CHRC_TX_UUID_FIFTH)
#define BT_UUID_PAAR_CHRC_TX BT_UUID_DECLARE_128(BT_UUID_PAAR_CHRC_TX_VAL)
#define BT_UUID_PAAR_CHRC_RX_VAL                         \
    BT_UUID_128_ENCODE(BT_UUID_PAAR_CHRC_RX_UUID_FIRST,  \
                       BT_UUID_PAAR_CHRC_RX_UUID_SECOND, \
                       BT_UUID_PAAR_CHRC_RX_UUID_THIRD,  \
                       BT_UUID_PAAR_CHRC_RX_UUID_FOURTH, \
                       BT_UUID_PAAR_CHRC_RX_UUID_FIFTH)
#define BT_UUID_PAAR_CHRC_RX BT_UUID_DECLARE_128(BT_UUID_PAAR_CHRC_RX_VAL)

/** @brief UBINOS device's uuid definitions */
#define BT_UUID_UBINOS_SERVICE_UART_VAL                   \
    BT_UUID_128_ENCODE(BT_UUID_UBINOS_SERVICE_UUID_FIRST, \
                       BT_UUID_PAAR_SERVICE_UUID_SECOND,  \
                       BT_UUID_PAAR_SERVICE_UUID_THIRD,   \
                       BT_UUID_PAAR_SERVICE_UUID_FOURTH,  \
                       BT_UUID_PAAR_SERVICE_UUID_FIFTH)
#define BT_UUID_UBINOS_SERVICE_UART BT_UUID_DECLARE_128(BT_UUID_UBINOS_SERVICE_UART_VAL)
#define BT_UUID_UBINOS_RX_CHRC_VAL                        \
    BT_UUID_128_ENCODE(BT_UUID_UBINOS_CHRC_RX_UUID_FIRST, \
                       BT_UUID_PAAR_CHRC_TX_UUID_SECOND,  \
                       BT_UUID_PAAR_CHRC_TX_UUID_THIRD,   \
                       BT_UUID_PAAR_CHRC_TX_UUID_FOURTH,  \
                       BT_UUID_PAAR_CHRC_TX_UUID_FIFTH)
#define BT_UUID_UBINOS_RX_CHRC BT_UUID_DECLARE_128(BT_UUID_UBINOS_RX_CHRC_VAL)
#define BT_UUID_UBINOS_TX_CHRC_VAL                        \
    BT_UUID_128_ENCODE(BT_UUID_UBINOS_CHRC_TX_UUID_FIRST, \
                       BT_UUID_PAAR_CHRC_RX_UUID_SECOND,  \
                       BT_UUID_PAAR_CHRC_RX_UUID_THIRD,   \
                       BT_UUID_PAAR_CHRC_RX_UUID_FOURTH,  \
                       BT_UUID_PAAR_CHRC_RX_UUID_FIFTH)
#define BT_UUID_UBINOS_TX_CHRC BT_UUID_DECLARE_128(BT_UUID_UBINOS_TX_CHRC_VAL)

// struct bt_paar_client
// {
//     uint16_t start_handle;
//     uint16_t end_handle;
//     uint16_t paar_tx_handle;
//     uint16_t paar_rx_handle;

//     struct bt_gatt_subscribe_params paar_tx_sub_params;
//     struct bt_gatt_discover_params  paar_tx_discover_params;
//     struct bt_gatt_subscribe_params paar_rx_sub_params;
//     struct bt_gatt_discover_params  paar_rx_discover_params;
//     struct bt_gatt_write_params     paar_write_params;
//     struct bt_gatt_read_params      paar_read_params;
//     // struct bt_paar_client_cb        *cb;
// };