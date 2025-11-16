#include "ble.h"

#define SOUND_UUID_SERVICE      0x0500
#define SOUND_UUID_CHAR_MODEL   0x0501
#define SOUND_UUID_CHAR_FEATURE 0x0502

/** UUID of the sound prediction service */
#define BT_UUID_SOUND_SERVICE_VAL \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + SOUND_UUID_SERVICE, BT_ADLD_SPECIFIC_UUID_SECOND, BT_ADLD_SPECIFIC_UUID_THIRD, BT_ADLD_SPECIFIC_UUID_FOURTH, BT_ADLD_SPECIFIC_UUID_LAST)

/** UUID of the sound model update characteristic */
#define BT_UUID_CHRC_SOUND_MODEL_VAL \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + SOUND_UUID_CHAR_MODEL, BT_ADLD_SPECIFIC_UUID_SECOND, BT_ADLD_SPECIFIC_UUID_THIRD, BT_ADLD_SPECIFIC_UUID_FOURTH, BT_ADLD_SPECIFIC_UUID_LAST)

/** UUID of the sound prediction feature collection characteristic */
#define BT_UUID_CHRC_SOUND_FEATURE_VAL \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + SOUND_UUID_CHAR_FEATURE, BT_ADLD_SPECIFIC_UUID_SECOND, BT_ADLD_SPECIFIC_UUID_THIRD, BT_ADLD_SPECIFIC_UUID_FOURTH, BT_ADLD_SPECIFIC_UUID_LAST)

/** Sound Prediction Service */
#define BT_UUID_SOUND_SERVICE       BT_UUID_DECLARE_128(BT_UUID_SOUND_SERVICE_VAL)
/** Sound Prediction Event Characteristic */
#define BT_UUID_CHRC_SOUND_MODEL    BT_UUID_DECLARE_128(BT_UUID_CHRC_SOUND_MODEL_VAL)
/** Sound Prediction Feature Collection Characteristic*/
#define BT_UUID_CHRC_SOUND_FEATURE  BT_UUID_DECLARE_128(BT_UUID_CHRC_SOUND_FEATURE_VAL)

/** BLE gatt attribute idx*/
#define SOUND_ATTRS_MODEL_IDX         1
#define SOUND_ATTRS_FEATURE_IDX       4

bool is_feature_notify_enabled(void);
bool is_model_notify_enabled(void);

uint8_t get_data_collection_mode(void);
struct k_sem *get_noti_enabled_sem(void);

int bt_sound_notify_model(const void *data, uint16_t len);
int bt_sound_notify_feature(const void *data, uint16_t len);

uint8_t bt_sound_result_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);
