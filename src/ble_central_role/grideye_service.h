#include "ble.h"

/** Grideye service UUID definitions*/
#define GRIDEYE_UUID_SERVICE            0x0200
#define GRIDEYE_UUID_CHAR_PREDICTION    0x0201
#define GRIDEYE_UUID_CHAR_RAW           0x0202
/** @brief GridEye Prediction Service UUID */
#define BT_UUID_GRIDEYE_SERVICE_VAL                                        \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + GRIDEYE_UUID_SERVICE, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                       \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                        \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                       \
                       BT_ADLD_SPECIFIC_UUID_LAST)

/** @brief GridEye Prediction Characteristic UUID */
#define BT_UUID_CHRC_GRIDEYE_PREDICTION_VAL                                        \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + GRIDEYE_UUID_CHAR_PREDICTION, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                               \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                                \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                               \
                       BT_ADLD_SPECIFIC_UUID_LAST)

/** @brief GridEye Prediction Raw Data Streaming Characteristic UUID */
#define BT_UUID_CHRC_GRIDEYE_RAW_STREAMING_VAL                              \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + GRIDEYE_UUID_CHAR_RAW, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                        \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                         \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                        \
                       BT_ADLD_SPECIFIC_UUID_LAST)

#define BT_UUID_GRIDEYE_SERVICE BT_UUID_DECLARE_128(BT_UUID_GRIDEYE_SERVICE_VAL)
#define BT_UUID_CHRC_GRIDEYE_PREDICTION BT_UUID_DECLARE_128(BT_UUID_CHRC_GRIDEYE_PREDICTION_VAL)
#define BT_UUID_CHRC_GRIDEYE_RAW_STREAMING BT_UUID_DECLARE_128(BT_UUID_CHRC_GRIDEYE_RAW_STREAMING_VAL)

/** @brief Callback type for grideye prediction is over and request to send ble notify to host. */
typedef void (*grideye_dir_cb_t)(const uint8_t grideye_dir);

/** @brief Callback type for grideye raw data streaming request. */
typedef void (*grideye_raw_cb_t)(const int *buf);

/** @brief Callback struct used by Grideye Prediction Service */
struct bt_grideye_cb
{
    grideye_dir_cb_t grideye_dir_cb;
    grideye_raw_cb_t grideye_raw_cb;
};

/** @brief Grideye Data Type */
struct bt_grideye_data_type
{
    uint8_t index;
    int data;
};

/** @brief Intialize the Grideye Prediction Service.
 *
 * This function registers a GATT service with two Characteristics
 * -> grideye prediction result
 * -> grideye raw data streaming
 *
 * Send notifications for the grideye prediction result \
 * to let connceted peers know where the user is moving direction while grideye is watching.
 * Write the grideye's raw data to connected peers for future analysis of grideye data.
 *
 * @param[in] callbacks Struct containing pointers to callback functions
 *          used by the service. This pointer can be NULL if no callback functions are defined.
 *
 * @retval  0 if the operation was successful.
 *          Otherwise, a (negative) error code is retruned.
 */
int bt_grideye_prediction_service_init(struct bt_grideye_cb *callback);

/** @brief Send the grideye's prediction result.
 *
 * This function sends a grideye's prediction result.
 *
 * @param[in] grideye_dir is the result of prediction.
 *
 * @retval  0 if the operation was successful.
 *          Otherwise, a (negative) error code is retruned.
 */
int bt_grideye_send_prediction_result(uint8_t grideye_dir);

/** @brief Streaming the grideye's raw data.
 *
 * This function sends a grideye's raw data to connected peers.
 * Because Grideye has 8x8 2d array sensor data, for each symbol has index of raw data and the raw data itself.
 * Grideye's maximum frame rate is 10fps so this function will called every 100ms if the service is enabled.
 *
 * @param[in] bt_grideye_data is structure of grideye data.
 * @param[in] index is grideye raw data's index.
 * @param[in] data is grideye's raw data itself.
 *
 * @retval  0 if the operation was successful.
 *          Otherwise, a (negative) error code is retruned.
 */
int bt_grideye_send_raw_data(struct bt_grideye_data_type *bt_grideye_data);

/**
 * @brief Get grideye service's CCC status
 * 
 * @return int "1" if the CCC is enabled, "0" if the CCC is disabled.
 */
int is_grideye_notify_enabled(void);