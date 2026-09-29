#ifndef BLE_APP_H
#define BLE_APP_H

#include <zephyr/bluetooth/conn.h>

/* bt_enable(), connection callbacks, advertising (audio + SMP/OTA UUIDs).
 * Call once from main(). Returns 0 or a negative error code. */
int ble_app_init(void);

/* Current connection, or NULL when not connected. */
struct bt_conn *ble_app_get_conn(void);

#endif /* BLE_APP_H */