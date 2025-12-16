#include <zephyr/drivers/gpio.h>

int button_init(gpio_callback_handler_t button_pressed);
int get_button_status();

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

int led_init();
int led_set(int color, int value);
int led_toggle(int color);
int blink_led(int color, int times);

void system_reboot();
