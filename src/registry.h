#pragma once
#include <stddef.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>

typedef struct {
    bt_addr_le_t addr;
    struct bt_conn *conn;
} control_block_t;

int register_connection(struct bt_conn *conn);
control_block_t *get_control_block(struct bt_conn *conn);
control_block_t *get_control_block_by_index(int index);
int unregister_connection(struct bt_conn *conn);
