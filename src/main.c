#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_relay_control.h"
#include "ui.h"

/* 위 파일의 프로토타입 */
// int central_discovery_start(void);

void main(void)
{
    printk("Relay Central discovery-subscribe start\n");

    int err = led_init();
    if (err) {
        /* LED init failed: show RED 3 times (best-effort). */
        (void)blink_led(RED_LED, 3);
    } else {
        /* Power-on indication: slower RGB sweep + WHITE flash. */
        (void)blink_led(RED_LED, 1);
        k_sleep(K_MSEC(150));
        (void)blink_led(GREEN_LED, 1);
        k_sleep(K_MSEC(150));
        (void)blink_led(BLUE_LED, 1);
        k_sleep(K_MSEC(150));

        (void)led_set(RED_LED, 1);
        (void)led_set(GREEN_LED, 1);
        (void)led_set(BLUE_LED, 1);
        k_sleep(K_MSEC(350));
        (void)led_set(RED_LED, 0);
        (void)led_set(GREEN_LED, 0);
        (void)led_set(BLUE_LED, 0);
        k_sleep(K_MSEC(200));
    }

    (void)ui_status_init();

    ble_relay_control_start();

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
