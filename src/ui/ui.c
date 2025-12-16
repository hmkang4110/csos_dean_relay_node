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

void system_reboot()
{
	// printk("SYSTEM REBOOT START in 1 seconds\n");
	// stm_write_bytes(STM_CMD_RESET, NULL, 0);

	sys_reboot(SYS_REBOOT_COLD);
}