#ifndef _DEAN_DEVICE_H_
#define _DEAN_DEVICE_H_

#include <zephyr/drivers/sensor.h>

#define GRID_EYE_PIXEL_SIZE                     64
#define GRID_EYE_ROW_SIZE                       8
#define GRID_EYE_COL_SIZE                       8
#define GRID_EYE_FIRST_DIR_FLAG_YES             1
#define GRID_EYE_FIRST_DIR_FLAG_NO              0
#define GRID_EYE_TRACK_SIZE                     30

#define GRID_EYE_WINDOW_SIZE                    4
#define GRID_EYE_STDDEV_THRESHOLD               600
#define GRID_EYE_NORMALIZED_STDDEV_THRESHOLD    0.02
#define GRID_EYE_TRIGGER_COUNT_THRESHOLD        4  // 열원이 감지되려면 최소 3개 이상 변화해야 함

//Ceiling Installed Ver.
#define GRID_EYE_DIFF_BUF_SIZE                  4
#define GRID_EYE_FRAME_SIZE		                10
#define GRID_EYE_PIXEL_DIFF_COUNT_THRESHOLD		5
#define GRID_EYE_TEMP_DIFF_THRESHOLD		    125
#define GRID_EYE_TEMP_DIFF_THRESHOLD_NEW	    150
#define GRID_EYE_PIXEL_ABSOL_COUNT_THRESHOLD	15
#define GRID_EYE_STDEV_THRESHOLD		        0.625
#define GRID_EYE_HEAT_SENSEND_THRESHOLD_CNT		5       // last value was 10
#define GRID_EYE_HEAT_SENSED_CELL_COUNT         6
#define GRID_EYE_HOT_PIXEL_THRESHOLD_VAL        1.0
#define GRID_EYE_QUEUE_MAX_SIZE			        200
#define GRID_EYE_HEAT_SENSE_MAX_SIZE            100

#define GRID_EYE_HEAT_NOISE_FILTERING_THRESHOLD  2
#define GRID_EYE_HEAT_SENSE_INFERENCE_THRESHOLD_SIZE    30
#define GRID_EYE_HEAT_CONTINUOUS_CHECK_THRESHOLD        200

#define GRID_EYE_HEAT_SENSED	1
#define GRID_EYE_NOTHING_HEAT	0

#define BH1745_READ_COUNT   0

typedef struct 
{
    float x;
    float y;    
} grideye_coordinate_t;

struct sensor_value;
int init_grideye_device(void);
int print_grideye_sensor_value(void);
int init_thread_dean_device(void);
void grideye_prediction_callback(const void *buf, uint16_t len);

#endif