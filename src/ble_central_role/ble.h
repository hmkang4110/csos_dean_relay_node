#ifndef _BLE_SERVICE_H_
#define _BLE_SERVICE_H_

#include <zephyr/bluetooth/uuid.h>

/** DE&N device specific uuids */
#define BT_ADLD_SPECIFIC_UUID_FIRST     0x4eab0000
#define BT_ADLD_SPECIFIC_UUID_SECOND    0x6bef
#define BT_ADLD_SPECIFIC_UUID_THIRD     0x11ee
#define BT_ADLD_SPECIFIC_UUID_FOURTH    0xb962
#define BT_ADLD_SPECIFIC_UUID_LAST      0x10012002809a

#define BT_ADLD_BASE_RESET_CHAR         0x0001

/** @brief DE&N base service uuid definitions*/
#define BT_UUID_BASE_SERVICE_VAL \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST, BT_ADLD_SPECIFIC_UUID_SECOND, BT_ADLD_SPECIFIC_UUID_THIRD, BT_ADLD_SPECIFIC_UUID_FOURTH, BT_ADLD_SPECIFIC_UUID_LAST)

#define BT_UUID_CHRC_RESET_VAL \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + BT_ADLD_BASE_RESET_CHAR, BT_ADLD_SPECIFIC_UUID_SECOND, BT_ADLD_SPECIFIC_UUID_THIRD, BT_ADLD_SPECIFIC_UUID_FOURTH, BT_ADLD_SPECIFIC_UUID_LAST)

#define BT_UUID_BASE_SERVICE BT_UUID_DECLARE_128(BT_UUID_BASE_SERVICE_VAL)
#define BT_UUID_CHRC_RESET   BT_UUID_DECLARE_128(BT_UUID_CHRC_RESET_VAL)

int ble_peripheral_adv_start(void);
int ble_peripheral_adv_stop(void);

int ble_central_scan_start(void);
int ble_central_scan_stop(void);

int init_ble(void);

extern bool ble_central_scanning;

#define MAC_ADDR_STR_LEN 17
extern char mac_addr_str[];

#endif