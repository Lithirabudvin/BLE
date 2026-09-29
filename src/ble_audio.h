#ifndef BLE_AUDIO_H
#define BLE_AUDIO_H

#include <stdbool.h>
#include <zephyr/bluetooth/uuid.h>

#include "audio_capture.h"

#define BT_UUID_AUDIO_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456789abc)
#define BT_UUID_AUDIO_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456789abd)

/* True while a client is connected and has enabled notifications. */
bool ble_audio_streaming(void);

/* Queue one PCM block for the sender thread.
 * Returns 0, -ENOTCONN (nobody listening) or -ENOMEM (queue full). */
int ble_audio_send(const struct pcm_block *blk);

/*
 * Notification format (each packet):
 *   bytes 0..1 : uint16 little-endian sequence number
 *   bytes 2..n : int16 little-endian samples (mono, 16 kHz)
 */

#endif /* BLE_AUDIO_H */