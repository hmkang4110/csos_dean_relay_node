#include "ble.h"

/** peripheral service UUID definitions*/
#define peripheral_UUID_SERVICE        0x0300
#define peripheral_UUID_CHAR_ACTION    0x0301
/** @brief peripheral action data Service UUID */
#define BT_UUID_peripheral_ACTION_SERVICE_VAL                                 \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + peripheral_UUID_SERVICE, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                   \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                    \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                   \
                       BT_ADLD_SPECIFIC_UUID_LAST)

/** @brief peripheral action data Characteristic UUID */
#define BT_UUID_CHRC_peripheral_ACTION_DATA_VAL                                   \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + peripheral_UUID_CHAR_ACTION, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                       \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                        \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                       \
                       BT_ADLD_SPECIFIC_UUID_LAST)

#define BT_UUID_peripheral_ACTION_SERVICE BT_UUID_DECLARE_128(BT_UUID_peripheral_ACTION_SERVICE_VAL)
#define BT_UUID_CHRC_peripheral_ACTION_DATA BT_UUID_DECLARE_128(BT_UUID_CHRC_peripheral_ACTION_DATA_VAL)



/* 타깃 디바이스 광고 이름 (Bluefruit.setName("BUTTON_RECORD_ULP")) */
#define TARGET_NAME "BUTTON_"
/* 타깃 디바이스 광고 이름은 설치되는 디바이스 이름 마다 서로 다르게 표현 */
/* */

/* Nordic UART Service (Adafruit BLEUart) 128-bit UUID */
static struct bt_uuid_128 NUS_SVC_UUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)
);

/* (옵션) TX/RX 캐릭터리스틱 UUID — 디스커버리/구독 시 사용 */
static struct bt_uuid_128 NUS_TX_UUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)
);
static struct bt_uuid_128 NUS_RX_UUID = BT_UUID_INIT_128(
    BT_UUID_128_ENCODE(0x6E400002, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA9E)
);

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif