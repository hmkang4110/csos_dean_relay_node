#pragma once

#include <zephyr/bluetooth/uuid.h>

/* 스캔 시작/정지 및 타깃 디바이스 이름 설정 */
int central_scan_set_target_name(const char *name);  /* NULL이면 전체 출력 */
int central_scan_start(void);
int central_scan_stop(void);
int ble_relay_control_start(void);

/* Helper to forward write requests to DEAN */
int ble_relay_send_write_to_dean(const struct bt_uuid *uuid, const void *data, uint16_t len);
