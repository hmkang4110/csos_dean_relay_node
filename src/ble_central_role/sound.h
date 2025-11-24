#ifndef SOUND_H
#define SOUND_H

#include "ble.h"
#include "mel_filter.h"

/** Sound Mode*/
#define SOUND_SERVICE_MODE 0
#define SOUND_FEATURE_COLLECTION_MODE 1
#define SOUND_MODEL_UPDATE_MODE 2
#define SOUND_MODEL_WAIT_MODE 3

/** Init sound service thread*/
int sound_thread_init(void);

// BLE model update cmd
#define BLE_MODEL_UPDATE_CMD_START 1
#define BLE_MODEL_UPDATE_CMD_DATA 2
#define BLE_MODEL_UPDATE_CMD_END 3
#define BLE_MODEL_UPDATE_CMD_REMOVE 4
#define BLE_MODEL_UPDATE_CMD_FAILED 11

#define BLE_FEATURE_COLLECTION_CMD_START 5
#define BLE_FEATURE_COLLECTION_CMD_DATA 6
#define BLE_FEATURE_COLLECTION_CMD_FINISH 7
#define BLE_FEATURE_COLLECTION_CMD_END 8

#define BLE_SOUND_MODEL_FRAME_SIZE 128

struct ble_model_update_packet
{
    uint8_t cmd;
    uint16_t seq;
    uint8_t data[BLE_SOUND_MODEL_FRAME_SIZE];
} __attribute__((packed));

struct ble_sound_feature_packet
{
    uint8_t cmd;
    uint16_t seq;
    uint16_t data[NUM_MEL_FILTERS];
} __attribute__((packed));

struct ble_ack_packet
{
    uint8_t cmd;
    uint16_t seq;
} __attribute__((packed));

bool is_sound_service_mode();
void process_model_update_write(const void *buf, uint16_t len);

struct wav_header_t
{
    uint8_t riff[4];
    uint32_t size;
    uint8_t wave[4];
    uint8_t fmt[4];
    uint32_t fmt_size;
    uint16_t format;
    uint16_t channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    uint8_t data[4];
    uint32_t data_size;
};

int sound_config_init(void);
int sound_model_update_from_sd(void);

#endif