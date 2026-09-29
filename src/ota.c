#include <zephyr/kernel.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/logging/log.h>

#include "ota.h"

LOG_MODULE_REGISTER(ota, LOG_LEVEL_INF);

int ota_init(void)
{
    /* Confirm this image. If you would rather only confirm after the
     * firmware has proven itself (e.g. audio started OK), move this call
     * to that point so a bad update can still roll back. */
    int err = boot_write_img_confirmed();

    if (err) {
        LOG_ERR("boot_write_img_confirmed failed: %d", err);
        return err;
    }

    LOG_INF("Image confirmed, OTA (SMP over BLE) ready");
    return 0;
}