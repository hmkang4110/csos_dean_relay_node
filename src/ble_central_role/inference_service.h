#include "ble.h"
#include "zephyr/drivers/sensor.h"

/** Environment service UUID definitions*/
#define INFERENCE_UUID_SERVICE                      0x0900
#define INFERENCE_UUID_CHAR_RAWDATA                 0x0901
#define INFERENCE_UUID_CHAR_SEQ_ANAL_RESULT         0x0902
#define INFERENCE_UUID_CHAR_DEBUG_STRING            0x0903
/** @brief Inference Result Send Service UUID */
#define BT_UUID_INFERENCE_SERVICE_VAL                                        \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + INFERENCE_UUID_SERVICE, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                   \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                    \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                   \
                       BT_ADLD_SPECIFIC_UUID_LAST)
/** @brief Inference RAW Data Send Service Characteristic UUID */
#define BT_UUID_CHRC_INFERENCE_RAWDATA_VAL                                              \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + INFERENCE_UUID_CHAR_RAWDATA, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                           \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                            \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                           \
                       BT_ADLD_SPECIFIC_UUID_LAST)
/** @brief Inference Result Send Service Characteristic UUID */
#define BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT_VAL                                   \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + INFERENCE_UUID_CHAR_SEQ_ANAL_RESULT, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                    \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                     \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                    \
                       BT_ADLD_SPECIFIC_UUID_LAST)
/** @brief Inference Debug String Send Service Characteristic UUID */
#define BT_UUID_CHRC_INFERENCE_DEBUG_STRING_VAL                                   \
    BT_UUID_128_ENCODE(BT_ADLD_SPECIFIC_UUID_FIRST + INFERENCE_UUID_CHAR_DEBUG_STRING, \
                       BT_ADLD_SPECIFIC_UUID_SECOND,                    \
                       BT_ADLD_SPECIFIC_UUID_THIRD,                     \
                       BT_ADLD_SPECIFIC_UUID_FOURTH,                    \
                       BT_ADLD_SPECIFIC_UUID_LAST)


#define BT_UUID_INFERENCE_SERVICE                   BT_UUID_DECLARE_128(BT_UUID_INFERENCE_SERVICE_VAL)
#define BT_UUID_CHRC_INFERENCE_RAWDATA              BT_UUID_DECLARE_128(BT_UUID_CHRC_INFERENCE_RAWDATA_VAL)
#define BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT      BT_UUID_DECLARE_128(BT_UUID_CHRC_INFERENCE_SEQ_ANAL_RESULT_VAL)
#define BT_UUID_CHRC_INFERENCE_DEBUG_STRING         BT_UUID_DECLARE_128(BT_UUID_CHRC_INFERENCE_DEBUG_STRING_VAL)


#define SENSOR_VALUE_PARAM_NUM 9

#define INFERENCE_RESULT_PACKET_SIZE                44
#define INFERENCE_RESULT_PACKET_TYPE_IDX_GRIDEYE    0
#define INFERENCE_RESULT_PACKET_DATA_IDX_GRIDEYE    1
#define INFERENCE_RESULT_PACKET_SIZE_GRIDEYE        1
#define INFERENCE_RESULT_PACKET_TYPE_IDX_ENV        2
#define INFERENCE_RESULT_PACKET_DATA_IDX_ENV        3
#define INFERENCE_RESULT_PACKET_SIZE_ENV            20
#define INFERENCE_RESULT_PACKET_TYPE_IDX_SOUND      23
#define INFERENCE_RESULT_PACKET_DATA_IDX_SOUND      24
#define INFERENCE_RESULT_PACKET_SIZE_SOUND_MAX      20

#define SOUND_LABEL_NUM 16  // 16 labels for sound classification

#define INFERENCE_RESULT_EXIST   1
#define INFERENCE_RESULT_NONE    0

/**
 * @brief Send Inference Result.
 * 
 * This function sends inference result to connected peers.
 * According to the packet type, 40bytes of the packet is encoded.
 * 
 * @param result_arr is the raw format of the MSGQ packet. It's size is various according to the MSGQ type.
 * @return int  
 */
int bt_inference_rawdata_send(uint8_t *packet_arr);

/** 
 * @brief Send Inference Result.
 * 
 * This function sends inference result to connected peers.
 * According to the sequence analysis result, @param result_len_uint16_t bytes of the packet will be sent.
 * The packet is string data, so it should be converted to char array when the central device receives it.
 * 
 * @param result_char_arr is string format of the sequence analysis result.
 * @param result_len_uint16_t is the length of the result_char_arr.
 * @return int
 */
int bt_inference_seq_anal_result_send(char *result_char_arr, uint16_t result_len_uint16_t);

/**
 * @brief Send Inference debugging string data.
 * 
 * This function sends inference debugging string data to connected peers.
 * JLinkRTT log data is sent to the SLiM Hub. This function will make the debugging service more efficient.
 * 
 * @param debug_string_arr is the debugging string data.
 * @param debug_string_len_uint16_t is the length of the debug_string_arr.
 * @return int
 */
int bt_inference_debug_string_send(char *debug_string_arr, uint16_t debug_string_len_uint16_t);

bool is_inference_notify_enabled(void);
bool is_inference_seq_anal_result_notify_enabled(void);
bool is_inference_debug_string_notify_enabled(void);