#pragma once

#include "ble_relay_control.h"

void relay_router_init(void);
void relay_router_attach_dean_handles(struct relay_session *session,
                                      uint16_t rawdata,
                                      uint16_t seq_result,
                                      uint16_t debug_str);
