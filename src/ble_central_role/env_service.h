#include "ble.h"
#include "zephyr/drivers/sensor.h"

/** Environment service UUID definitions*/
#define ENV_UUID_SERVICE            0x0400
#define ENV_UUID_CHAR_PREDICTION    0x0401
#define ENV_UUID_CHAR_RAW           0x0402
/** @brief bme680 Environmental Sensor Data Send Service UUID */
#define BT_UUID_ENV_SERVICE_VAL                                        \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + ENV_UUID_SERVICE, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                   \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                    \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                   \
                       BT_ADLD_SPECIFIC_UUID_LAST)
/** @brief bme680 Environmental Sensor Data Send Service Characteristic UUID */
#define BT_UUID_CHRC_ENV_SEND_VAL                                              \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + ENV_UUID_CHAR_PREDICTION, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                           \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                            \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                           \
                       BT_ADLD_SPECIFIC_UUID_LAST)
#define BT_UUID_CHRC_ENV_RESERVED_VAL                                   \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + ENV_UUID_CHAR_RAW, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                    \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                     \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                    \
                       BT_ADLD_SPECIFIC_UUID_LAST)

#define BT_UUID_ENV_SERVICE BT_UUID_DECLARE_128(BT_UUID_ENV_SERVICE_VAL)
#define BT_UUID_CHRC_ENV_SEND BT_UUID_DECLARE_128(BT_UUID_CHRC_ENV_SEND_VAL)
#define BT_UUID_CHRC_ENV_RESERVED BT_UUID_DECLARE_128(BT_UUID_CHRC_ENV_RESERVED_VAL)

// struct sensor_value;
/**
 * @brief Send Environmental Sensor RAW data.
 *
 * This function sends environmental sensor data to connected peers.
 * These are RAW data of bme688's environmental data. No Algorithm-processed result is provided.
 *
 * @param raw_press is atomospheric pressure
 * @param raw_temp is ambient temperature
 * @param raw_humid is relative humidity of indoor-place
 * @param raw_gas_res is gas sensor's resistance which changes according to heating time and indoor air quality.
 * @param IAQ is index of Air Quality which generated from bsec2 library
 * @param static_IAQ is IAQ value which is recommended to stationary device.
 * @param e_CO2 is estimated CO2 Equivalent value.
 * @param B_VOC is Breathe-VOC value.
 * @param gas_percent is gas pecentage value relatively how bad the air quality among calibrated time.
 *          If scan rate is 0.33Hz, calibration time is 3d, if 0.0033Hz, calibration time id 28d
 * @return 0 if the operatoin was successful.
 *         Otherwise, a (negative) error code is returned.
 */
int bt_env_send_float(float raw_press, float raw_temp, float raw_humid, float raw_gas_res,
                      float IAQ, float statis_IAQ, float e_CO2, float B_VOC, float gas_precent);

int bt_env_send_int(struct sensor_value raw_press, struct sensor_value raw_temp,
                    struct sensor_value raw_humid, struct sensor_value raw_gas_res,
                    struct sensor_value IAQ, struct sensor_value static_IAQ,
                    struct sensor_value e_CO2, struct sensor_value B_VOC,
                    struct sensor_value gas_percent);


uint8_t bt_env_value_notify_handler(struct bt_conn *conn, struct bt_gatt_subscribe_params *params, const void *data, uint16_t length);