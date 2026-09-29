#ifndef OTA_H
#define OTA_H

#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

/* SMP service UUID, advertised so nRF Device Manager can find the device. */
#define OTA_SMP_UUID_VAL  SMP_BT_SVC_UUID_VAL

/* Call after ble_app_init(). Marks the running image as good so MCUboot
 * doesn't roll back. The SMP/mcumgr service itself is registered
 * automatically by CONFIG_MCUMGR_TRANSPORT_BT. Returns 0 or -errno. */
int ota_init(void);

#endif /* OTA_H */