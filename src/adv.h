#pragma once

#include <zephyr/bluetooth/bluetooth.h>

void start_adv(const struct bt_le_adv_param *param);
void stop_adv(void);