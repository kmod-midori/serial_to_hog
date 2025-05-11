#include "registry.h"
#include "zephyr/bluetooth/addr.h"
#include "zephyr/bluetooth/conn.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(registry);

static control_block_t control_blocks[CONFIG_BT_MAX_CONN] = {0};

int register_connection(struct bt_conn *conn) {
  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
    if (control_blocks[i].conn == NULL) {
      control_blocks[i].conn = conn;
      const bt_addr_le_t *addr = bt_conn_get_dst(conn);
      bt_addr_le_copy(&control_blocks[i].addr, addr);

      char addr_s[BT_ADDR_LE_STR_LEN];
      bt_addr_le_to_str(addr, addr_s, sizeof(addr_s));
      LOG_INF("Assigned connection %d to %s", i, addr_s);

      return i;
    }
  }

  return -1; // No available control block
}

control_block_t *get_control_block(struct bt_conn *conn) {
  const bt_addr_le_t *addr = bt_conn_get_dst(conn);

  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
    if (control_blocks[i].conn != NULL && bt_addr_le_cmp(addr, &control_blocks[i].addr) == 0) {
      return &control_blocks[i];
    }
  }

  return NULL; // Connection not found
}

control_block_t *get_control_block_by_index(int index) {
  if (index < 0 || index >= CONFIG_BT_MAX_CONN) {
    return NULL; // Invalid index
  }

  control_block_t *cb = &control_blocks[index];
  if (cb->conn == NULL) {
    return NULL; // Control block not initialized
  }

  return &control_blocks[index];
}

int unregister_connection(struct bt_conn *conn) {
  control_block_t *cb = get_control_block(conn);
  if (cb) {
    char addr_s[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&cb->addr, addr_s, sizeof(addr_s));

    memset(cb, 0, sizeof(control_block_t));

    LOG_INF("Unregistered connection %s", addr_s);
  }
  return -1; // Connection not found
}