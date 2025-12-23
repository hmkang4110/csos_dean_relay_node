/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <inttypes.h>

#include <zephyr/sys/reboot.h>

// #include "stm_support.h"
// #include "stm_packet.h"
#include "ui.h"

#define SLEEP_TIME_MS	1

enum ui_status_event {
	UI_EVT_DEAN_CONNECTED = 1,
	UI_EVT_SLIMHUB_CONNECTED,
	UI_EVT_DEAN_DISCONNECTED,
	UI_EVT_SLIMHUB_DISCONNECTED,
	UI_EVT_RELAY_ACTIVITY,
};

K_MSGQ_DEFINE(ui_evt_msgq, sizeof(uint8_t), 16, 1);

#define UI_THREAD_STACK_SIZE 1024
/* Keep UI thread at the lowest app priority so it never interferes with BLE. */
#define UI_THREAD_PRIORITY   K_LOWEST_APPLICATION_THREAD_PRIO
K_THREAD_STACK_DEFINE(ui_thread_stack, UI_THREAD_STACK_SIZE);
static struct k_thread ui_thread_data;

static uint32_t last_relay_evt_ms;
#define RELAY_ACTIVITY_COOLDOWN_MS 250

static void set_rgb(bool r, bool g, bool b)
{
	led_set(RED_LED, r ? 1 : 0);
	led_set(GREEN_LED, g ? 1 : 0);
	led_set(BLUE_LED, b ? 1 : 0);
}

static void blink_rgb(bool r, bool g, bool b, int times, int on_ms, int off_ms)
{
	for (int i = 0; i < times; i++) {
		set_rgb(r, g, b);
		k_sleep(K_MSEC(on_ms));
		set_rgb(false, false, false);
		k_sleep(K_MSEC(off_ms));
	}
}

static void ui_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);

	uint8_t evt;
	while (1) {
		if (k_msgq_get(&ui_evt_msgq, &evt, K_FOREVER) != 0) {
			continue;
		}

		switch ((enum ui_status_event)evt) {
		case UI_EVT_DEAN_CONNECTED:          /* GREEN */
			blink_rgb(false, true, false, 2, 220, 180);
			break;
		case UI_EVT_SLIMHUB_CONNECTED:       /* BLUE */
			blink_rgb(false, false, true, 2, 220, 180);
			break;
		case UI_EVT_DEAN_DISCONNECTED:       /* RED x1 */
			blink_rgb(true, false, false, 1, 300, 200);
			break;
		case UI_EVT_SLIMHUB_DISCONNECTED:    /* RED x2 */
			blink_rgb(true, false, false, 2, 300, 200);
			break;
		case UI_EVT_RELAY_ACTIVITY:          /* YELLOW (R+G) */
			blink_rgb(true, true, false, 1, 90, 0);
			break;
		default:
			break;
		}
	}
}

static void ui_post_evt(uint8_t evt)
{
	/* Drop if queue is full; status indications are best-effort. */
	(void)k_msgq_put(&ui_evt_msgq, &evt, K_NO_WAIT);
}

/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE	DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static struct gpio_callback button_cb_data;

int button_init(gpio_callback_handler_t button_handler)
{
	int ret;

	if (!device_is_ready(button.port)) {
        printk("Error: button device %s is not ready\n", button.port->name);
        return -1;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
        return ret;
	}

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
        return ret;
	}

	gpio_init_callback(&button_cb_data, button_handler, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);
	printk("Set up button at %s pin %d\n", button.port->name, button.pin);
    
    return ret;
}

int get_button_status()
{
	return gpio_pin_get_dt(&button);
}

static struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static struct gpio_dt_spec green_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0});
static struct gpio_dt_spec blue_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0});

int led_init()
{
	int ret;

	if (red_led.port && !device_is_ready(red_led.port)) {
		printk("Error: LED device %s is not ready; ignoring it\n", red_led.port->name);
		red_led.port = NULL;
	}
	if (red_led.port) {
		ret = gpio_pin_configure_dt(&red_led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n", 
			       ret, red_led.port->name, red_led.pin);
			red_led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", red_led.port->name, red_led.pin);
		}
	}
	if (green_led.port && !device_is_ready(green_led.port)) {
		printk("Error: LED device %s is not ready; ignoring it\n", green_led.port->name);
		green_led.port = NULL;
	}
	if (green_led.port) {
		ret = gpio_pin_configure_dt(&green_led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n", 
			       ret, green_led.port->name, green_led.pin);
			green_led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", green_led.port->name, green_led.pin);
		}
	}
	if (blue_led.port && !device_is_ready(blue_led.port)) {
		printk("Error: LED device %s is not ready; ignoring it\n", blue_led.port->name);
		blue_led.port = NULL;
	}
	if (blue_led.port) {
		ret = gpio_pin_configure_dt(&blue_led, GPIO_OUTPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure LED device %s pin %d\n", 
			       ret, blue_led.port->name, blue_led.pin);
			blue_led.port = NULL;
		} else {
			printk("Set up LED at %s pin %d\n", blue_led.port->name, blue_led.pin);
		}
	}

	/* Ensure LEDs start OFF. */
	if (red_led.port) {
		gpio_pin_set_dt(&red_led, 0);
	}
	if (green_led.port) {
		gpio_pin_set_dt(&green_led, 0);
	}
	if (blue_led.port) {
		gpio_pin_set_dt(&blue_led, 0);
	}

	return 0;
}

int led_set(int color, int value)
{
	switch(color) 
	{
	case RED_LED:
		return gpio_pin_set_dt(&red_led, value);
	case GREEN_LED:
		return gpio_pin_set_dt(&green_led, value);
	case BLUE_LED:
		return gpio_pin_set_dt(&blue_led, value);
	default:
		return -1;
	}
}

int led_toggle(int color)
{
	switch(color)
	{
	case RED_LED:
		return gpio_pin_toggle_dt(&red_led);
	case GREEN_LED:
		return gpio_pin_toggle_dt(&green_led);
	case BLUE_LED:
		return gpio_pin_toggle_dt(&blue_led);
	default:
		return -1;
	}
}

int blink_led(int color, int times)
{
	for (int i = 0; i < times; i++) {
		led_set(color, 1);
		k_sleep(K_MSEC(100));
		led_set(color, 0);
		k_sleep(K_MSEC(100));
	}
	return 0;
}

int ui_status_init(void)
{
	last_relay_evt_ms = 0;
	k_thread_create(&ui_thread_data, ui_thread_stack, UI_THREAD_STACK_SIZE,
			ui_thread_fn, NULL, NULL, NULL,
			UI_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ui_thread_data, "ui_status");
	return 0;
}

void ui_status_dean_connected(void)
{
	uint8_t evt = UI_EVT_DEAN_CONNECTED;
	ui_post_evt(evt);
}

void ui_status_slimhub_connected(void)
{
	uint8_t evt = UI_EVT_SLIMHUB_CONNECTED;
	ui_post_evt(evt);
}

void ui_status_dean_disconnected(void)
{
	uint8_t evt = UI_EVT_DEAN_DISCONNECTED;
	ui_post_evt(evt);
}

void ui_status_slimhub_disconnected(void)
{
	uint8_t evt = UI_EVT_SLIMHUB_DISCONNECTED;
	ui_post_evt(evt);
}

void ui_status_relay_activity(void)
{
	uint32_t now = k_uptime_get_32();
	if ((now - last_relay_evt_ms) < RELAY_ACTIVITY_COOLDOWN_MS) {
		return;
	}
	last_relay_evt_ms = now;

	uint8_t evt = UI_EVT_RELAY_ACTIVITY;
	ui_post_evt(evt);
}

void system_reboot()
{
	// printk("SYSTEM REBOOT START in 1 seconds\n");
	// stm_write_bytes(STM_CMD_RESET, NULL, 0);

	sys_reboot(SYS_REBOOT_COLD);
}
