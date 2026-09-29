#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "audio_capture.h"
#include "ble_app.h"
#include "ble_audio.h"
#include "ota.h" 

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define LED_BLINK_MS 200

static const struct gpio_dt_spec led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static void led_timer_fn(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    gpio_pin_toggle_dt(&led);
}
K_TIMER_DEFINE(led_timer, led_timer_fn, NULL);

int main(void)
{
    int err;

    if (gpio_is_ready_dt(&led)) {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
        k_timer_start(&led_timer, K_MSEC(LED_BLINK_MS), K_MSEC(LED_BLINK_MS));
    } else {
        LOG_ERR("LED GPIO not ready");
    }

    /* BLE first: OTA keeps working even if audio fails to start. */
    err = ble_app_init();
    if (err) {
        LOG_ERR("BLE init failed: %d", err);
        return err;
    }

    err = ota_init();
    if (err) {
        LOG_WRN("OTA init failed: %d", err);
    }

    err = audio_capture_init();
    if (err) {
        LOG_ERR("Audio capture init failed: %d (BLE/OTA still running)", err);
        return 0;
    }

    static struct pcm_block blk;

    while (1) {
        if (audio_capture_get_block(&blk, K_FOREVER) != 0) {
            continue;
        }

        if (ble_audio_streaming() && ble_audio_send(&blk) == -ENOMEM) {
            LOG_WRN("PCM queue full, block dropped");
        }
    }

    return 0;
}