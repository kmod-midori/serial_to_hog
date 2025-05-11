#ifndef HOG_H
#define HOG_H

#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>

#define HID_RPT_ID_KEY_IN 0x02
#define HID_RPT_ID_LED_OUT 0x02
#define HID_RPT_ID_MOUSE_IN 0x01
#define HID_RPT_ID_CC_IN 0x03

void hog_init(void);
int hog_notify(struct bt_conn *conn, uint8_t report_id, uint8_t *data, uint8_t len);

#endif /* HOG_H */
