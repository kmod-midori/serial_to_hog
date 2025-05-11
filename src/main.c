/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "adv.h"
#include "hog.h"
#include "registry.h"
#include "zephyr/bluetooth/addr.h"
#include "zephyr/sys/ring_buffer.h"

LOG_MODULE_REGISTER(main);

const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(serial_to_hogp_serial));
RING_BUF_DECLARE(uart_rx_buf, 2048);
RING_BUF_DECLARE(uart_tx_buf, 2048);

struct k_work start_adv_worker;
static void start_adv_handler(struct k_work *work) {
  stop_adv();
  start_adv();
}

typedef struct {
  bool connected;
  uint16_t conn_id;
  bt_addr_t addr;
} __attribute__((packed)) uart_connected_device_t;
struct k_work send_connections_worker;
static void send_connections_handler(struct k_work *work) {
  uart_connected_device_t devices[CONFIG_BT_MAX_CONN];
  memset(devices, 0, sizeof(devices));

  int count = 0;

  for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
    control_block_t *cb = get_control_block_by_index(i);
    if (cb) {
      devices[i].connected = true;
      devices[i].conn_id = i;
      bt_addr_copy(&devices[i].addr, &cb->addr.a);
      count++;
    }
  }

  uint8_t preamble[] = {1, 2, 4, 5};
  ring_buf_put(&uart_tx_buf, preamble, sizeof(preamble));
  ring_buf_put(&uart_tx_buf, (uint8_t *)&devices, sizeof(devices));
  uart_irq_tx_enable(uart_dev);

  LOG_INF("Sending %d connected devices", count);
}

static void connected(struct bt_conn *conn, uint8_t err) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (err) {
    LOG_ERR("Failed to connect to %s, err 0x%02x %s", addr, err, bt_hci_err_to_str(err));
    return;
  }

  LOG_INF("Connected to %s", addr);

  err = bt_conn_set_security(conn, BT_SECURITY_L2);
  if (err) {
    LOG_ERR("Failed to set security (err %d)", err);
  }

  register_connection(conn);

  err = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(6, 10, 0, 10));
  if (err) {
    LOG_ERR("Failed to update connection parameters (err %d)", err);
    return;
  }

  k_work_submit(&start_adv_worker);
  k_work_submit(&send_connections_worker);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {

  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("Disconnected from %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));

  unregister_connection(conn);

  k_work_submit(&start_adv_worker);
  k_work_submit(&send_connections_worker);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

  if (!err) {
    LOG_INF("Security changed: %s level %u", addr, level);
  } else {
    LOG_ERR("Security failed: %s level %u err %s(%d)", addr, level, bt_security_err_to_str(err), err);
  }
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout) {
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  LOG_INF("LE parameters updated: %s interval %u latency %u timeout %u", addr, interval, latency, timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
    .security_changed = security_changed,
    .le_param_updated = le_param_updated,
};

static void bt_ready(int err) {
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return;
  }

  k_work_init(&start_adv_worker, start_adv_handler);

  LOG_INF("Bluetooth ready");

  hog_init();

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  k_work_submit(&start_adv_worker);
}

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

typedef union {
  struct {
    uint8_t command;
    uint8_t padding[3];
  } any;
  struct {
    uint8_t command;
    uint8_t conn_id;
    uint8_t report_id;
    uint8_t report_len;
  } send_report;
} command_header_t;

#define REPORT_LEN_MAX (8)
typedef struct {
  uint8_t conn_id;
  uint8_t report_id;
  uint8_t report_len;
  uint8_t report[REPORT_LEN_MAX];
} report_message_t;

K_MSGQ_DEFINE(report_msgq, sizeof(report_message_t), 128, 1);

static void uart_interrupt_handler(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);

  while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
    if (uart_irq_rx_ready(dev)) {
      uint8_t local_buf[64];

      int recv_len = uart_fifo_read(dev, local_buf, sizeof(local_buf));
      if (recv_len > 0) {
        ring_buf_put(&uart_rx_buf, local_buf, recv_len);
      }

      while (ring_buf_size_get(&uart_rx_buf) >= sizeof(command_header_t)) {
        command_header_t header;
        ring_buf_peek(&uart_rx_buf, (uint8_t *)&header, sizeof(command_header_t));

        switch (header.any.command) {
        case 0xF0: {
          uint8_t report_len = MIN(header.send_report.report_len, REPORT_LEN_MAX);
          uint8_t combined_len = sizeof(command_header_t) + report_len;
          if (ring_buf_size_get(&uart_rx_buf) >= combined_len) {
            // We have enough data, drop the header
            ring_buf_get(&uart_rx_buf, NULL, sizeof(command_header_t));

            report_message_t report_msg = {
                .conn_id = header.send_report.conn_id,
                .report_id = header.send_report.report_id,
                .report_len = report_len,
            };
            ring_buf_get(&uart_rx_buf, report_msg.report, report_len);
            k_msgq_put(&report_msgq, &report_msg, K_NO_WAIT); // Silently drop if full
          }
          break;
        }
        case 0xF1:
          // Request connected devices
          // Drop the header
          ring_buf_get(&uart_rx_buf, NULL, sizeof(command_header_t));

          // Send the connected devices
          k_work_submit(&send_connections_worker);
          break;
        default:
          // Drop the header
          ring_buf_get(&uart_rx_buf, NULL, sizeof(command_header_t));
          break;
        }
      }
    }

    if (uart_irq_tx_ready(dev)) {
      uint8_t local_buf[64];
      int rb_len;

      rb_len = ring_buf_get(&uart_tx_buf, local_buf, sizeof(local_buf));
      if (rb_len == 0) {
        uart_irq_tx_disable(dev);
      } else {
        uart_fifo_fill(dev, local_buf, rb_len);
        if (ring_buf_is_empty(&uart_tx_buf)) {
          // No more data to send, disable TX interrupt
          // otherwise it will keep sending empty data
          uart_irq_tx_disable(dev);
        }
      }
    }
  }
}

int main(void) {
  int err;

  err = usb_enable(NULL);
  if (err) {
    return 0;
  }

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
    return 0;
  }

  err = uart_irq_callback_set(uart_dev, uart_interrupt_handler);
  if (err) {
    LOG_ERR("Failed to set UART IRQ (err %d)", err);
    return 0;
  }
  uart_irq_rx_enable(uart_dev);
  k_work_init(&send_connections_worker, send_connections_handler);

  err = bt_enable(bt_ready);
  if (err) {
    LOG_ERR("Bluetooth init failed (err %d)", err);
    return 0;
  }

  err = bt_conn_auth_cb_register(&auth_cb_display);
  if (err) {
    LOG_ERR("Failed to register auth callbacks (err %d)", err);
    return 0;
  }

  while (1) {
    report_message_t report_msg;
    k_msgq_get(&report_msgq, &report_msg, K_FOREVER);
    control_block_t *cb = get_control_block_by_index(report_msg.conn_id);
    if (cb) {
      hog_notify(cb->conn, report_msg.report_id, report_msg.report, report_msg.report_len);
    } else {
      LOG_ERR("Connection %d not found", report_msg.conn_id);
    }
  }

  return 0;
}
