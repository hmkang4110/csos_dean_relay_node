#pragma once

#include "ble_relay_control.h"

typedef void (*relay_scan_match_cb)(const struct dean_adv_report *report);

int relay_scan_start(relay_scan_match_cb cb);
void relay_scan_stop(void);
