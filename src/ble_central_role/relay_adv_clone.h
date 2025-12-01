#pragma once

#include "ble_relay_control.h"

int relay_adv_clone_start(struct relay_session *session);
void relay_adv_clone_stop(struct relay_session *session);
