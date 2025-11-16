#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_relay_control.h"

/* 위 파일의 프로토타입 */
// int central_discovery_start(void);

void main(void)
{
    printk("Relay Central discovery-subscribe start\n");
    ble_relay_control_start();

    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
