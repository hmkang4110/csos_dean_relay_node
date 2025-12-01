#pragma once

#include "ble_relay_control.h"

void relay_mapping_init(void);
struct relay_session *relay_mapping_alloc(const struct dean_adv_report *report);
struct relay_session *relay_mapping_by_dean_conn(struct bt_conn *conn);
struct relay_session *relay_mapping_by_hub_conn(struct bt_conn *conn);
struct relay_session *relay_mapping_find_empty(void);
void relay_mapping_release(struct relay_session *session);
