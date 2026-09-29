/*
 * Audio GATT service: one notify characteristic streaming raw PCM.
 * A dedicated thread splits each block into MTU-sized notifications.
 * The service and thread are defined statically, so there is no init call.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "ble_app.h"
#include "ble_audio.h"

LOG_MODULE_REGISTER(ble_audio, LOG_LEVEL_INF);

#define QUEUE_DEPTH  8
#define MAX_PAYLOAD  240      /* max sample bytes per notification */

static struct bt_uuid_128 audio_service_uuid =
    BT_UUID_INIT_128(BT_UUID_AUDIO_SERVICE_VAL);
static struct bt_uuid_128 audio_char_uuid =
    BT_UUID_INIT_128(BT_UUID_AUDIO_CHAR_VAL);

K_MSGQ_DEFINE(pcm_q, sizeof(struct pcm_block), QUEUE_DEPTH, 4);

static volatile bool notify_enabled;
static uint16_t seq;

static void audio_ccc_cfg_changed(const struct bt_gatt_attr *attr,
                                  uint16_t value)
{
    ARG_UNUSED(attr);

    bool enable = (value == BT_GATT_CCC_NOTIFY);

    if (enable) {
        seq = 0;
        k_msgq_purge(&pcm_q);
    }
    notify_enabled = enable;
    LOG_INF("Notifications %s", enable ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(audio_svc,
    BT_GATT_PRIMARY_SERVICE(&audio_service_uuid),
    BT_GATT_CHARACTERISTIC(&audio_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(audio_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void audio_disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);
    notify_enabled = false;
}

BT_CONN_CB_DEFINE(audio_conn_callbacks) = {
    .disconnected = audio_disconnected,
};

bool ble_audio_streaming(void)
{
    return notify_enabled && (ble_app_get_conn() != NULL);
}

int ble_audio_send(const struct pcm_block *blk)
{
    if (!ble_audio_streaming()) {
        return -ENOTCONN;
    }
    if (k_msgq_put(&pcm_q, blk, K_NO_WAIT) != 0) {
        return -ENOMEM;
    }
    return 0;
}

static void sender_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    static struct pcm_block blk;
    uint8_t pkt[2 + MAX_PAYLOAD];

    while (1) {
        k_msgq_get(&pcm_q, &blk, K_FOREVER);

        const uint8_t *p = (const uint8_t *)blk.s;
        size_t left = AUDIO_BLOCK_BYTES;

        while (left > 0) {
            struct bt_conn *conn = ble_app_get_conn();

            if (conn == NULL || !notify_enabled) {
                break;                      /* drop rest of this block */
            }

            uint16_t mtu = bt_gatt_get_mtu(conn);
            size_t max_payload = (mtu > 5U) ? (size_t)(mtu - 3U - 2U) : 0U;

            max_payload = MIN(max_payload, (size_t)MAX_PAYLOAD);
            max_payload &= ~1U;             /* whole 16-bit samples only */

            if (max_payload == 0U) {
                k_msleep(10);
                continue;
            }

            size_t n = MIN(left, max_payload);

            sys_put_le16(seq, pkt);
            memcpy(&pkt[2], p, n);

            int err = bt_gatt_notify(conn, &audio_svc.attrs[2], pkt, n + 2);

            if (err == -ENOMEM) {
                k_msleep(2);                /* TX buffers full, retry */
                continue;
            }
            if (err) {
                LOG_WRN("bt_gatt_notify failed: %d", err);
                break;
            }

            seq++;
            p += n;
            left -= n;
        }
    }
}

K_THREAD_DEFINE(sender_tid, 4096, sender_thread, NULL, NULL, NULL, 7, 0, 0);