/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include "hog.h"
#include "zephyr/logging/log.h"

LOG_MODULE_REGISTER(hog);

enum {
  HIDS_INPUT = 0x01,
  HIDS_OUTPUT = 0x02,
  HIDS_FEATURE = 0x03,
};

static uint8_t report_map[] = {
    0x05, 0x01,              // Usage Page (Generic Desktop)        0
    0x09, 0x06,              // Usage (Keyboard)                    2
    0xa1, 0x01,              // Collection (Application)            4
    0x85, HID_RPT_ID_KEY_IN, //  Report ID (2)                      6
    // Modifier
    0x95, 0x08, //  Report Count (8)                   8
    0x75, 0x01, //  Report Size (1)                    10
    0x15, 0x00, //  Logical Minimum (0)                12
    0x25, 0x01, //  Logical Maximum (1)                14
    0x05, 0x07, //  Usage Page (Keyboard)              16
    0x19, 0xe0, //  Usage Minimum (224)                18
    0x29, 0xe7, //  Usage Maximum (231)                20
    0x81, 0x02, //  Input (Data,Var,Abs)               22
    // Main Keys
    0x95, 0x06,       //  Report Count (6)                   24
    0x75, 0x08,       //  Report Size (8)                    26
    0x15, 0x00,       //  Logical Minimum (0)                28
    0x26, 0xff, 0x00, //  Logical Maximum (255)              30
    0x05, 0x07,       //  Usage Page (Keyboard)              33
    0x19, 0x00,       //  Usage Minimum (0)                  35
    0x2a, 0xff, 0x00, //  Usage Maximum (255)                37
    0x81, 0x00,       //  Input (Data,Arr,Abs)               40
    // LED Output (5+3)
    0x85, HID_RPT_ID_LED_OUT, //  Report ID (2)                      42
    0x05, 0x08,               //  Usage Page (LEDs)                  44
    0x95, 0x05,               //  Report Count (5)                   46
    0x75, 0x01,               //  Report Size (1)                    48
    0x15, 0x00,               //  Logical Minimum (0)                50
    0x25, 0x01,               //  Logical Maximum (1)                52
    0x19, 0x01,               //  Usage Minimum (1)                  54
    0x29, 0x05,               //  Usage Maximum (5)                  56
    0x91, 0x02,               //  Output (Data,Var,Abs)              58
    0x95, 0x01,               //  Report Count (1)                   60
    0x75, 0x03,               //  Report Size (3)                    62
    0x91, 0x01,               //  Output (Cnst,Arr,Abs)              64
    0xc0,                     // End Collection                      66

    0x05, 0x01,                // Usage Page (Generic Desktop)        67
    0x09, 0x02,                // Usage (Mouse)                       69
    0xa1, 0x01,                // Collection (Application)            71
    0x85, HID_RPT_ID_MOUSE_IN, //  Report ID (1)                      73
    0x09, 0x01,                //  Usage (Pointer)                    75
    0xa1, 0x00,                //  Collection (Physical)              77
    // 16 Buttons
    0x05, 0x09, //   Usage Page (Button)               79
    0x19, 0x01, //   Usage Minimum (1)                 81
    0x29, 0x10, //   Usage Maximum (16)                83
    0x15, 0x00, //   Logical Minimum (0)               85
    0x25, 0x01, //   Logical Maximum (1)               87
    0x95, 0x10, //   Report Count (16)                 89
    0x75, 0x01, //   Report Size (1)                   91
    0x81, 0x02, //   Input (Data,Var,Abs)              93
    // X/Y Motion
    0x05, 0x01,       //   Usage Page (Generic Desktop)      95
    0x16, 0x01, 0x80, //   Logical Minimum (-32767)          97
    0x26, 0xff, 0x7f, //   Logical Maximum (32767)           100
    0x75, 0x10,       //   Report Size (16)                  103
    0x95, 0x02,       //   Report Count (2)                  105
    0x09, 0x30,       //   Usage (X)                         107
    0x09, 0x31,       //   Usage (Y)                         109
    0x81, 0x06,       //   Input (Data,Var,Rel)              111
    0x15, 0x81,       //   Logical Minimum (-127)            113
    0x25, 0x7f,       //   Logical Maximum (127)             115
    // Vertical Wheel
    0x75, 0x08, //   Report Size (8)                   117
    0x95, 0x01, //   Report Count (1)                  119
    0x09, 0x38, //   Usage (Wheel)                     121
    0x81, 0x06, //   Input (Data,Var,Rel)              123
    // Horizontal Wheel
    0x05, 0x0c,             //   Usage Page (Consumer Devices)     125
    0x0a, 0x38, 0x02,       //   Usage (AC Pan)                    127
    0x95, 0x01,             //   Report Count (1)                  130
    0x81, 0x06,             //   Input (Data,Var,Rel)              132
    0xc0,                   //  End Collection                     134
    0xc0,                   // End Collection                      135
    0x05, 0x0c,             // Usage Page (Consumer Devices)       136
    0x09, 0x01,             // Usage (Consumer Control)            138
    0xa1, 0x01,             // Collection (Application)            140
    0x85, HID_RPT_ID_CC_IN, //  Report ID (3)                      142
    // 2 * 16-bit usage IDs
    0x75, 0x10,       //  Report Size (16)                   144
    0x95, 0x02,       //  Report Count (2)                   146
    0x15, 0x01,       //  Logical Minimum (1)                148
    0x26, 0xff, 0x02, //  Logical Maximum (767)              150
    0x19, 0x01,       //  Usage Minimum (1)                  153
    0x2a, 0xff, 0x02, //  Usage Maximum (767)                155
    0x81, 0x00,       //  Input (Data,Arr,Abs)               158
    0xc0,             // End Collection                      160
};

typedef struct {
  uint16_t len;
  void *buf;
} fixed_read_info;

#define FIXED_READ_INFO(_info_name, _name)                                                                             \
  fixed_read_info _info_name = {                                                                                       \
      .len = sizeof(_name),                                                                                            \
      .buf = &_name,                                                                                                   \
  }

struct hids_info {
  uint16_t version; /* version number of base USB HID Specification */
  uint8_t code;     /* country HID Device hardware is localized for. */
  uint8_t flags;
} __packed;
enum {
  HIDS_REMOTE_WAKE = BIT(0),
  HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

static struct hids_info hid_info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE,
};
FIXED_READ_INFO(hid_info_read_info, hid_info);
FIXED_READ_INFO(hid_report_map_read_info, report_map);

struct hids_report {
  uint8_t id;   /* report id */
  uint8_t type; /* report type */
} __packed;
static struct hids_report mouse_ref = {
    .id = HID_RPT_ID_MOUSE_IN,
    .type = HIDS_INPUT,
};
FIXED_READ_INFO(mouse_ref_read_info, mouse_ref);
static struct hids_report keyboard_ref = {
    .id = HID_RPT_ID_KEY_IN,
    .type = HIDS_INPUT,
};
FIXED_READ_INFO(keyboard_ref_read_info, keyboard_ref);
static struct hids_report cc_ref = {
    .id = HID_RPT_ID_CC_IN,
    .type = HIDS_INPUT,
};
FIXED_READ_INFO(cc_ref_read_info, cc_ref);
static struct hids_report led_ref = {
    .id = HID_RPT_ID_LED_OUT,
    .type = HIDS_OUTPUT,
};
FIXED_READ_INFO(led_ref_read_info, led_ref);

static ssize_t fixed_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                          uint16_t offset) {
  fixed_read_info *info = attr->user_data;
  if (info == NULL) {
    return 0;
  }

  char uuid[BT_UUID_STR_LEN];
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  bt_uuid_to_str(attr->uuid, uuid, sizeof(uuid));
  LOG_DBG("Read %s %u@%u from %s", uuid, len, offset, addr);

  return bt_gatt_attr_read(conn, attr, buf, len, offset, info->buf, info->len);
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value) {
  char uuid[BT_UUID_STR_LEN];
  bt_uuid_to_str(attr->uuid, uuid, sizeof(uuid));
  LOG_DBG("CCC changed %s to %u", uuid, value);
}

static ssize_t read_noop(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len,
                         uint16_t offset) {
  char uuid[BT_UUID_STR_LEN];
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  bt_uuid_to_str(attr->uuid, uuid, sizeof(uuid));
  LOG_DBG("Read %s %u@%u from %s", uuid, len, offset, addr);

  return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t write_noop(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len,
                          uint16_t offset, uint8_t flags) {
  char uuid[BT_UUID_STR_LEN];
  char addr[BT_ADDR_LE_STR_LEN];
  bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
  bt_uuid_to_str(attr->uuid, uuid, sizeof(uuid));
  LOG_DBG("Write %s %u@%u from %s", uuid, len, offset, addr);

  return len;
}

#define SAMPLE_BT_PERM_READ BT_GATT_PERM_READ_ENCRYPT
#define SAMPLE_BT_PERM_WRITE BT_GATT_PERM_WRITE_ENCRYPT

static struct bt_gatt_ccc_managed_user_data mouse_ccc =
    BT_GATT_CCC_MANAGED_USER_DATA_INIT(input_ccc_changed, NULL, NULL);
static struct bt_gatt_ccc_managed_user_data keyboard_ccc =
    BT_GATT_CCC_MANAGED_USER_DATA_INIT(input_ccc_changed, NULL, NULL);
static struct bt_gatt_ccc_managed_user_data cc_ccc = BT_GATT_CCC_MANAGED_USER_DATA_INIT(input_ccc_changed, NULL, NULL);

#define HID_INPUT_REPORT_CHARACTERISTIC(_ccc, _ref_read_info)                                                          \
  BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT, BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY, SAMPLE_BT_PERM_READ, read_noop, \
                         NULL, &_ref_read_info),                                                                       \
      BT_GATT_CCC_MANAGED(&_ccc, SAMPLE_BT_PERM_READ | SAMPLE_BT_PERM_WRITE),                                          \
      BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ, fixed_read, NULL, &_ref_read_info)

#define HID_OUTPUT_REPORT_CHARACTERISTIC(_ref_read_info)                                                               \
  BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,                                                                          \
                         BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,                     \
                         SAMPLE_BT_PERM_READ | SAMPLE_BT_PERM_WRITE, read_noop, write_noop, NULL),                     \
      BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ, fixed_read, NULL, &_ref_read_info)

/* HID Service Declaration */
BT_GATT_SERVICE_DEFINE(hog_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),

                       HID_INPUT_REPORT_CHARACTERISTIC(mouse_ccc, mouse_ref_read_info),
                       HID_INPUT_REPORT_CHARACTERISTIC(keyboard_ccc, keyboard_ref_read_info),
                       HID_INPUT_REPORT_CHARACTERISTIC(cc_ccc, cc_ref_read_info),
                       HID_OUTPUT_REPORT_CHARACTERISTIC(led_ref_read_info),

                       BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, fixed_read, NULL,
                                              &hid_info_read_info),
                       BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, fixed_read,
                                              NULL, &hid_report_map_read_info),
                       BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT, BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                                              BT_GATT_PERM_WRITE, NULL, write_noop, NULL), );

static const struct bt_gatt_attr *mouse_attr;
static const struct bt_gatt_attr *keyboard_attr;
static const struct bt_gatt_attr *cc_attr;
static const struct bt_gatt_attr *led_attr;

void hog_init(void) {
  for (int i = 0; i < hog_svc.attr_count; i++) {
    const struct bt_gatt_attr *attr = &hog_svc.attrs[i];

    if (bt_uuid_cmp(attr->uuid, BT_UUID_HIDS_REPORT) != 0) {
      continue;
    }

    if (attr->user_data == &mouse_ref_read_info) {
      mouse_attr = attr;
    } else if (attr->user_data == &keyboard_ref_read_info) {
      keyboard_attr = attr;
    } else if (attr->user_data == &cc_ref_read_info) {
      cc_attr = attr;
    } else if (attr->user_data == &led_ref_read_info) {
      led_attr = attr;
    }
  }

  if (mouse_attr == NULL || keyboard_attr == NULL || cc_attr == NULL || led_attr == NULL) {
    LOG_ERR("Failed to find HID attributes");
  }

  LOG_INF("HID attributes found");
}

int hog_notify(struct bt_conn *conn, uint8_t report_id, uint8_t *data, uint8_t len) {
  switch (report_id) {
  case HID_RPT_ID_MOUSE_IN:
    return bt_gatt_notify(conn, mouse_attr, data, len);
    break;
  case HID_RPT_ID_KEY_IN:
    return bt_gatt_notify(conn, keyboard_attr, data, len);
    break;
  case HID_RPT_ID_CC_IN:
    return bt_gatt_notify(conn, cc_attr, data, len);
    break;
  default:
    LOG_ERR("Unknown report ID %u", report_id);
    return -EINVAL;
    break;
  }
}