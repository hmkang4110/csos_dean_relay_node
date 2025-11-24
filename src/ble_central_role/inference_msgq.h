#include <stdint.h>
#include <zephyr/kernel.h>

extern struct k_msgq inference_msgq;

enum inference_msgq_type
{
    INFERENCE_MSGQ_TYPE_ENV,
    INFERENCE_MSGQ_TYPE_GRIDEYE_RAW,
    INFERENCE_MSGQ_TYPE_GRIDEYE_FLAG,
    INFERENCE_MSGQ_TYPE_SOUND,
    INFERENCE_MSGQ_TYPE_PERIPHERAL_BUTTON,
    INFERENCE_MSGQ_TYPE_IFDATA_LOAD,
};

struct inference_msgq_packet_t
{
    uint8_t type;
    void *data;
};

struct inference_msgq_env_packet_t
{
    uint8_t type;
    float temperature;
    float humidity;
    float iaq;
    float eco2;
    float b_voc;
};

struct inference_msgq_grideye_packet_t
{
    uint8_t type;
    uint8_t direction;
};

struct inference_msgq_sound_packet_t
{
    uint8_t type;
    int8_t result[16];
};


typedef struct
{
    uint8_t type;
    union
    {
        uint8_t direction;
        struct
        {
            float temperature;
            float humidity;
            float iaq;
            float eco2;
            float b_voc;
        } env;
        struct
        {
            uint8_t size;
            int8_t result[20];
        } sound;
        struct
        {
            uint8_t adl_idx[4];
        } peripheral_button;
    };    
} inference_msgq_packet_data_t;
