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

/* High-level relay status indicators (non-blocking; implemented via UI worker thread). */
int ui_status_init(void);
void ui_status_dean_connected(void);
void ui_status_slimhub_connected(void);
void ui_status_dean_disconnected(void);
void ui_status_slimhub_disconnected(void);
void ui_status_relay_activity(void);

void system_reboot();
