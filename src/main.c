/*
 * Piezo audio capture -> ADPCM encode -> BLE GATT notify
 * Target : nRF52840 Feather Express, nRF Connect SDK v3.2.1 (Zephyr 4.x)
 *
 * ── Signal path ──────────────────────────────────────────────────────────
 * Piezo disk (analog, AIN1 / P0.03 by default)
 *   -> SAADC advanced mode, EasyDMA, hardware-timer-triggered, double-buffered
 *   -> IMA-ADPCM encoder  (bit-identical to the Dart AdpcmEncoder)
 *   -> BLE GATT NOTIFY    (same service/characteristic UUIDs as the Flutter apps)
 *
 * ── Changes from the original draft ─────────────────────────────────────
 * 1. nrfx_saadc_simple_mode_set  -> nrfx_saadc_advanced_mode_set
 *    PPI per-sample hardware triggering REQUIRES advanced mode.
 *    Simple mode fires all samples at once on a single trigger; it cannot
 *    accept individual sample tasks from a timer PPI chain.
 *
 * 2. nrfx_saadc_start()  -> nrfx_saadc_mode_trigger()
 *    nrfx_saadc_start() does not exist in nrfx v3 (the API used by NCS v2+).
 *    nrfx_saadc_mode_trigger() arms the driver to accept SAMPLE tasks and
 *    starts DMA into the first pre-queued buffer.
 *
 * 3. saadc_handler: buffer re-queue logic corrected for advanced mode.
 *    In advanced mode the driver has ALREADY switched to the second
 *    (pre-queued) buffer by the time EVT_DONE fires for the first one.
 *    The handler must therefore re-queue the buffer that JUST FINISHED
 *    (so it becomes available after the currently-running one completes),
 *    not the one that is already running.
 *
 * 4. Removed the unused start_advertising() helper (main() calls
 *    bt_le_adv_start() directly, silencing the -Wunused-function warning).
 *
 * ── Packet size ──────────────────────────────────────────────────────────
 * BLOCK_SAMPLES = 800  matches kFramesPerBlock in the Dart apps.
 * ADPCM packs 2 samples/byte -> 400 bytes/notification.
 * At 8000 Hz that is 10 notifications/second, well within BLE bandwidth.
 * 400 bytes fits in one ATT packet at the MTU=517 the Flutter app negotiates.
 *
 * ── Receiver app compatibility ───────────────────────────────────────────
 * No changes needed in the Flutter receiver.
 * - Same service/characteristic UUIDs.
 * - Same ADPCM encoding (nibble order, step/index tables).
 * - Same 400-byte notification size.
 * - The waveform and level-meter widgets are sample-stream agnostic.
 *   Mono 8 kHz looks correct on the display.
 *
 * ── ADJUST FOR YOUR HARDWARE ─────────────────────────────────────────────
 * PIEZO_AIN_PIN  : AIN pin your piezo signal wire is connected to.
 *                  Default = AIN1 (P0.03). Check your board's pinout.
 * SAMPLE_RATE_HZ : 8000 Hz works for most piezo pickups. Increase for
 *                  higher fidelity (watch BLE throughput).
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>

#include <nrfx.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_saadc.h>

LOG_MODULE_REGISTER(piezo_audio, LOG_LEVEL_INF);

/* ── Config ──────────────────────────────────────────────────────────────*/
#define PIEZO_AIN_PIN     NRF_SAADC_INPUT_AIN1  /* AIN1 = P0.03 on nRF52840 */
#define SAMPLE_RATE_HZ    8000
#define BLOCK_SAMPLES     800   /* must match kFramesPerBlock in Dart */
#define ADPCM_BLOCK_BYTES (BLOCK_SAMPLES / 2)   /* 400 bytes per notification */

/* ── UUIDs ── must match kAudioServiceUuid / kAudioCharUuid in Flutter ──*/
#define BT_UUID_AUDIO_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456789abc)
#define BT_UUID_AUDIO_CHAR_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x1234, 0x1234, 0x123456789abd)

static struct bt_uuid_128 audio_service_uuid =
    BT_UUID_INIT_128(BT_UUID_AUDIO_SERVICE_VAL);
static struct bt_uuid_128 audio_char_uuid =
    BT_UUID_INIT_128(BT_UUID_AUDIO_CHAR_VAL);

/* ── IMA-ADPCM tables — copied verbatim from AdpcmEncoder.dart ──────────*/
static const int16_t step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130,
    143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449,
    494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,
    4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767,
};
static const int8_t index_table[8] = { -1, -1, -1, -1, 2, 4, 6, 8 };

static int32_t adpcm_predictor  = 0;
static int32_t adpcm_step_index = 0;

static void adpcm_reset(void)
{
    adpcm_predictor  = 0;
    adpcm_step_index = 0;
}

/* Mirrors AdpcmEncoder._encodeSample() exactly. */
static uint8_t adpcm_encode_sample(int16_t sample)
{
    int32_t step  = step_table[adpcm_step_index];
    int32_t delta = sample - adpcm_predictor;
    uint8_t nibble = 0;

    if (delta < 0) { nibble = 8; delta = -delta; }

    if (delta >= step)         { nibble |= 4; delta -= step; }
    if (delta >= (step >> 1))  { nibble |= 2; delta -= (step >> 1); }
    if (delta >= (step >> 2))  { nibble |= 1; }

    int32_t vpdiff = step >> 3;
    if (nibble & 4) vpdiff += step;
    if (nibble & 2) vpdiff += step >> 1;
    if (nibble & 1) vpdiff += step >> 2;

    if (nibble & 8) { adpcm_predictor -= vpdiff; }
    else            { adpcm_predictor += vpdiff; }

    if (adpcm_predictor >  32767) adpcm_predictor =  32767;
    if (adpcm_predictor < -32768) adpcm_predictor = -32768;

    adpcm_step_index += index_table[nibble & 7];
    if (adpcm_step_index < 0)  adpcm_step_index = 0;
    if (adpcm_step_index > 88) adpcm_step_index = 88;

    return nibble;
}

/* Low nibble = first sample, high nibble = second — matches Dart encoder. */
static void adpcm_encode_block(const int16_t *pcm, uint8_t *out)
{
    for (int i = 0; i < ADPCM_BLOCK_BYTES; i++) {
        uint8_t lo = adpcm_encode_sample(pcm[i * 2]);
        uint8_t hi = adpcm_encode_sample(pcm[i * 2 + 1]);
        out[i] = (lo & 0x0F) | ((hi & 0x0F) << 4);
    }
}

/* ── SAADC double-buffered continuous capture ────────────────────────────*/
static int16_t saadc_buf_a[BLOCK_SAMPLES];
static int16_t saadc_buf_b[BLOCK_SAMPLES];
static uint8_t adpcm_out[ADPCM_BLOCK_BYTES];

static nrfx_timer_t sample_timer = NRFX_TIMER_INSTANCE(1);

/* PPI channel 0 — free at boot on a fresh nRF52840. If other peripherals
 * also use PPI, pick a different free channel number.
 */
static const nrf_ppi_channel_t ppi_channel = NRF_PPI_CHANNEL0;

static volatile bool  block_ready = false;
static int16_t       *ready_buf;   /* the buffer the DONE event just filled */

/* Timer ISR — not needed; PPI wires TIMER directly to SAADC SAMPLE task.
 * The handler must exist but can be empty.
 */
static void timer_handler(nrf_timer_event_t event_type, void *ctx)
{
    ARG_UNUSED(event_type);
    ARG_UNUSED(ctx);
}

/*
 * SAADC event handler.
 *
 * FIX 3 — buffer re-queue corrected for advanced mode:
 * When EVT_DONE fires for buffer X, the driver has ALREADY switched DMA to
 * the other buffer (Y, which was pre-queued before mode_trigger() was
 * called). To maintain continuous ping-pong, we must re-queue X (the one
 * that just finished) so it becomes available after Y completes.
 * The original code tried to re-queue Y, which is already running —
 * that produces a NRFX_ERROR_INVALID_STATE error and breaks ping-pong.
 */
static void saadc_handler(nrfx_saadc_evt_t const *event)
{
    if (event->type == NRFX_SAADC_EVT_DONE) {
        /* Point ready_buf at the buffer that just filled. */
        ready_buf = (int16_t *)event->data.done.p_buffer;

        /* Re-queue the buffer that just finished so the driver can use it
         * again after the currently-running buffer completes.
         * Do NOT queue the other buffer — it is already running.
         */
        nrfx_saadc_buffer_set(ready_buf, BLOCK_SAMPLES);

        block_ready = true;
    }
}

static void saadc_timer_ppi_init(void)
{
    uint32_t err;

    /* ── SAADC channel ───────────────────────────────────────────────── */
    LOG_INF("Configuring SAADC channel");

    nrfx_saadc_channel_t channel =
        NRFX_SAADC_DEFAULT_CHANNEL_SE(PIEZO_AIN_PIN, 0);

    channel.channel_config.gain      = NRF_SAADC_GAIN1_4;
    channel.channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
    channel.channel_config.acq_time  = NRF_SAADC_ACQTIME_10US;

    /* ── SAADC init ──────────────────────────────────────────────────── */
    LOG_INF("SAADC init");
    err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("SAADC init failed: 0x%08x", err);
        return;
    }

    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("SAADC channel config failed: 0x%08x", err);
        return;
    }

    /* ── FIX 1: advanced mode instead of simple mode ────────────────────
     * nrfx_saadc_advanced_mode_set() enables:
     *   - External per-sample hardware trigger via the SAMPLE task
     *     (which the PPI wires from the timer).
     *   - START_ON_END: when one DMA buffer completes, the driver
     *     automatically starts filling the next pre-queued buffer,
     *     guaranteeing zero-gap continuous capture.
     * Simple mode fires all samples at once on a single trigger and does
     * NOT support per-sample external triggering.
     */
    nrfx_saadc_adv_config_t adv_config =
        NRFX_SAADC_DEFAULT_ADV_CONFIG;

    adv_config.start_on_end = true;


    err = nrfx_saadc_advanced_mode_set(
        BIT(0),
        NRF_SAADC_RESOLUTION_12BIT,
        &adv_config,
        saadc_handler
    );


    LOG_INF("advanced mode ret=0x%08x", err);


    if (err != NRFX_SUCCESS)
    {
        LOG_ERR("SAADC advanced mode failed");
        return;
    }

    /* ── Queue both DMA buffers before starting ──────────────────────── */
    LOG_INF("Queuing DMA buffers");

    err = nrfx_saadc_buffer_set(saadc_buf_a, BLOCK_SAMPLES);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Buffer A queue failed: 0x%08x", err);
        return;
    }

    /* Second call queues buf_b as the "next" buffer; the driver will
     * automatically switch to it when buf_a completes (START_ON_END).
     */
    err = nrfx_saadc_buffer_set(saadc_buf_b, BLOCK_SAMPLES);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Buffer B queue failed: 0x%08x", err);
        return;
    }

    /* ── FIX 2: nrfx_saadc_mode_trigger() instead of nrfx_saadc_start()
     * nrfx_saadc_start() does not exist in nrfx v3.
     * nrfx_saadc_mode_trigger() arms the driver to accept SAMPLE tasks
     * and points the DMA engine at the first pre-queued buffer.
     * Actual sampling starts when the timer begins firing (below).
     */
    LOG_INF("SAADC mode trigger");
    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("SAADC trigger failed: 0x%08x", err);
        return;
    }

    /* ── Hardware timer at SAMPLE_RATE_HZ ────────────────────────────── */
    LOG_INF("Timer init");

    nrfx_timer_config_t timer_cfg = {
        .frequency          = NRF_TIMER_FREQ_1MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        .p_context          = NULL,
    };

    err = nrfx_timer_init(&sample_timer, &timer_cfg, timer_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Timer init failed: 0x%08x", err);
        return;
    }

    /* Compare value = 1 MHz / 8000 Hz = 125 ticks between samples. */
    uint32_t ticks = 1000000UL / SAMPLE_RATE_HZ;

    nrfx_timer_extended_compare(
        &sample_timer,
        NRF_TIMER_CC_CHANNEL0,
        ticks,
        NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
        false   /* no interrupt — PPI handles the trigger */
    );

    /* ── PPI: TIMER COMPARE0 event  ->  SAADC SAMPLE task ───────────── */
    LOG_INF("PPI setup");

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI,
        ppi_channel,
        nrfx_timer_compare_event_address_get(&sample_timer, NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE)
    );

    nrf_ppi_channel_enable(NRF_PPI, ppi_channel);

    /* ── Start the timer — sampling begins immediately ───────────────── */
    nrfx_timer_enable(&sample_timer);

    LOG_INF("SAADC running: %d Hz, block %d samples -> %d ADPCM bytes",
            SAMPLE_RATE_HZ, BLOCK_SAMPLES, ADPCM_BLOCK_BYTES);
}

/* ── BLE GATT service ────────────────────────────────────────────────────*/
static bool           notify_enabled = false;
static struct bt_conn *current_conn  = NULL;

static void audio_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    LOG_INF("Notifications %s", notify_enabled ? "enabled" : "disabled");
    if (notify_enabled) {
        adpcm_reset(); /* reset predictor state for a fresh stream */
    }
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

static void send_adpcm_block(uint8_t *data, uint16_t len)
{
    if (!current_conn || !notify_enabled) {
        return;
    }
    /* attrs[2] is the characteristic VALUE attribute:
     *   [0] = service declaration
     *   [1] = characteristic declaration
     *   [2] = characteristic value   <-- this is what bt_gatt_notify needs
     *   [3] = CCC descriptor
     */
    int err = bt_gatt_notify(current_conn, &audio_svc.attrs[2], data, len);
    if (err) {
        LOG_WRN("bt_gatt_notify failed: %d", err);
    }
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("Connected");
    current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason %u)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    notify_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* Advertise with the audio service UUID so the Flutter app can find us by
 * scanning with withServices: [Guid(kAudioServiceUuid)].
 */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_AUDIO_SERVICE_VAL),
};

/* ── Main ────────────────────────────────────────────────────────────────*/
int main(void)
{
    int err;

    LOG_INF("Piezo BLE audio starting");

    /* Bluetooth stack */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed: %d", err);
        return -1;
    }
    LOG_INF("Bluetooth ready");

    /* FIX 4: start_advertising() helper removed — call bt_le_adv_start
     * directly here as was already done in main(), silencing the
     * -Wunused-function warning from the original draft.
     */
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed: %d", err);
        return -1;
    }
    LOG_INF("Advertising started");

    /* SAADC + timer + PPI pipeline */
    saadc_timer_ppi_init();

    /* Main loop — wait for DMA buffers to fill, then encode and notify */
    uint32_t block_count = 0;

    while (1) {
        if (block_ready) {
            block_ready = false;

            /* Periodic ADC range log — useful for verifying the piezo
             * is connected and the signal is within ADC range.
             */
            if (++block_count % 10 == 0) {
                int16_t min_v = ready_buf[0], max_v = ready_buf[0];
                for (int i = 1; i < BLOCK_SAMPLES; i++) {
                    if (ready_buf[i] < min_v) min_v = ready_buf[i];
                    if (ready_buf[i] > max_v) max_v = ready_buf[i];
                }
                LOG_INF("ADC raw  min=%d  max=%d", min_v, max_v);
            }

            /* PCM -> ADPCM (400 bytes) */
            adpcm_encode_block(ready_buf, adpcm_out);

            /* BLE GATT notify */
            send_adpcm_block(adpcm_out, ADPCM_BLOCK_BYTES);
        }

        k_sleep(K_MSEC(5));
    }

    return 0;
}