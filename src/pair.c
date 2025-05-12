#include "pair.h"
#include "zephyr/bluetooth/bluetooth.h"
#include "zephyr/bluetooth/conn.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(pair);

static void bt_auth_cancel(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Pairing cancelled: %s", addr);
}

static void bt_pairing_confirm(struct bt_conn *conn) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Pairing confirm: %s", addr);

  int err = bt_conn_auth_pairing_confirm(conn);
  if (err) {
    LOG_ERR("Pairing confirm failed (err %d)", err);
    return;
  }

  LOG_INF("Pairing confirm success: %s", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
    .pairing_confirm = bt_pairing_confirm,
    .cancel = bt_auth_cancel,
};

int register_auth_callbacks(void) {
    return bt_conn_auth_cb_register(&auth_cb_display);
}