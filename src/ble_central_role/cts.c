/** @file
 *  @brief CTS Service sample
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <time.h>

#include "clock.h"
#include "cts.h"

static struct cts_datetime ct;
static uint8_t ct_update;

void time_to_cts()
{
	struct tm time;
	clock_get_datetime(&time);

	ct.year = time.tm_year + 1900; // Convert to full year
	ct.month = time.tm_mon + 1;	// Convert to 1-based month
	ct.day = time.tm_mday;
	ct.hours = time.tm_hour;
	ct.minutes = time.tm_min;
	ct.seconds = time.tm_sec;

	// Convert weekday: tm_wday (0=Sun, 6=Sat) → CTS format (1=Mon, 7=Sun)
	ct.day_of_week = (time.tm_wday == 0) ? 7 : time.tm_wday;

	ct.exact_time_256 = 0U;
	ct.adjust_reason = 0U;
}

void sync_cts_to_time()
{
	struct tm time;
	time.tm_year = ct.year - 1900; // Convert to tm_year format
	time.tm_mon = ct.month - 1;	   // Convert to 0-based month
	time.tm_mday = ct.day;
	time.tm_hour = ct.hours;
	time.tm_min = ct.minutes;
	time.tm_sec = ct.seconds;

	// Convert CTS weekday (1=Mon, 7=Sun) → tm_wday (0=Sun, 6=Sat)
	time.tm_wday = (ct.day_of_week == 7) ? 0 : ct.day_of_week;

	time.tm_yday = 0;
	time.tm_isdst = 0;

	clock_set_datetime(&time);
}

K_WORK_DEFINE(cts_sync_work, sync_cts_to_time);

static void ct_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	/* TODO: Handle value */
}

static ssize_t read_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
					   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &ct, sizeof(ct));
}

static ssize_t write_ct(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						const void *buf, uint16_t len, uint16_t offset,
						uint8_t flags)
{
	if (offset != 0 || len != sizeof(ct))
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&ct, buf, len);
	ct_update = 1U;

	k_work_submit(&cts_sync_work);
	return len;
}

/* Current Time Service Declaration */
BT_GATT_SERVICE_DEFINE(cts_cvs,
					   BT_GATT_PRIMARY_SERVICE(BT_UUID_CTS),
					   BT_GATT_CHARACTERISTIC(BT_UUID_CTS_CURRENT_TIME,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
											  read_ct, write_ct, &ct),
					   BT_GATT_CCC(ct_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), );

static void generate_current_time()
{
	uint16_t year;

	/* 'Exact Time 256' contains 'Day Date Time' which contains
	 * 'Date Time' - characteristic contains fields for:
	 * year, month, day, hours, minutes and seconds.
	 */

	year = sys_cpu_to_le16(2024);
	memcpy(&ct.year, &year, 2); /* year */
	ct.month = 1U;				/* months starting from 1 */
	ct.day = 1U;				/* day */
	ct.hours = 0U;				/* hours */
	ct.minutes = 00U;			/* minutes */
	ct.seconds = 00U;			/* seconds */

	/* 'Day of Week' part of 'Day Date Time' */
	ct.day_of_week = 1U; /* day of week starting from 1 */

	/* 'Fractions 256 part of 'Exact Time 256' */
	ct.exact_time_256 = 0U;

	/* Adjust reason */
	ct.adjust_reason = 0U; /* No update, change, etc */
}

void cts_init(void)
{
	/* Simulate current time for Current Time Service */
	generate_current_time();
}

void cts_notify(void)
{ /* Current Time Service updates only when time is changed */
	if (!ct_update)
	{
		return;
	}

	ct_update = 0U;
	bt_gatt_notify(NULL, &cts_cvs.attrs[1], &ct, sizeof(ct));
}

void notify_time_to_cts(struct tm *time)
{
	ct.year = time->tm_year + 1900;
	ct.month = time->tm_mon + 1;
	ct.day = time->tm_mday;
	ct.hours = time->tm_hour;
	ct.minutes = time->tm_min;
	ct.seconds = time->tm_sec;

	ct.day_of_week = (time->tm_wday == 0) ? 7 : time->tm_wday;
	ct.exact_time_256 = 0U;
	ct.adjust_reason = 0U;

	cts_notify();
}

int cts_write(struct bt_conn *conn, uint16_t handle)
{
	time_to_cts();

	return bt_gatt_write_without_response(conn, handle, &ct, sizeof(ct), false);
}
