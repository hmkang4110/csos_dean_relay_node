#pragma once
#include <stdint.h>

/* 스캔 시작/정지 및 타깃 디바이스 이름 설정 */
int central_scan_set_target_name(const char *name);  /* NULL이면 전체 출력 */
int central_scan_start(void);
int central_scan_stop(void);
int ble_relay_control_start(void);

/* Downstream forwarding to DEAN (MAC in big-endian order) */
int relay_forward_rawdata_to_dean(const uint8_t mac[6], const uint8_t *payload, uint16_t len);
int relay_forward_location_to_dean(const uint8_t mac[6], const uint8_t *payload, uint16_t len);
