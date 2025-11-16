/** @file
 *  @brief CTS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void cts_init(void);
void cts_notify(void);

struct cts_datetime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t day_of_week;
    uint8_t exact_time_256;
    uint8_t adjust_reason;
} __packed;

void time_to_cts();

void sync_cts_to_time();
int cts_write(struct bt_conn *conn, uint16_t handle);

#ifdef __cplusplus
}
#endif
