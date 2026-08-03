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
 * ── Fix history ────────────────────────────────────────────────────────
 * 1. nrfx_saadc_simple_mode_set  -> nrfx_saadc_advanced_mode_set
 *    PPI per-sample hardware triggering REQUIRES advanced mode.
 *
 * 2. nrfx_saadc_start()  -> nrfx_saadc_mode_trigger()
 *    nrfx_saadc_start() does not exist in this nrfx generation.
 *
 * 3. saadc_handler: buffer re-queue corrected for advanced mode — re-queue
 *    the buffer that JUST finished, not the one currently running.
 *
 * 4. Removed unused start_advertising() helper.
 *
 * 5. Manually IRQ_CONNECT the SAADC interrupt before nrfx_saadc_init() —
 *    we bypass Zephyr's CONFIG_ADC subsystem, so nothing else binds
 *    SAADC_IRQn into the vector table for us.
 *
 * 6. Removed a duplicate nrfx_saadc_init() call.
 *
 * 7. The SAADC driver in this nrfx generation returns plain `int`
 *    (0 = success, negative errno = failure) — NOT the old nrfx_err_t /
 *    NRFX_SUCCESS (0x0BAD0000) convention. All SAADC calls check
 *    `!= 0` accordingly. TIMER (below) still uses the classic
 *    nrfx_err_t / NRFX_SUCCESS convention — the two driver families
 *    genuinely disagree within the same SDK.
 *
 * 8. THE ACTUAL ROOT CAUSE of the timer assert:
 *    NRFX_TIMER_INSTANCE(reg) casts whatever you pass it directly to
 *    (NRF_TIMER_Type *) — it expects the peripheral's base-address
 *    symbol (NRF_TIMER2), NOT a bare instance number.
 *    NRFX_TIMER_INSTANCE(2) was casting the literal integer 2 to a
 *    pointer (address 0x00000002), which can never match any real
 *    TIMER peripheral's base address. This is why the assert failed
 *    identically regardless of any Kconfig/devicetree changes.
 *
 * ── Packet size ──────────────────────────────────────────────────────────
 * BLOCK_SAMPLES = 800  matches kFramesPerBlock in the Dart apps.
 * ADPCM packs 2 samples/byte -> 400 bytes/notification.
 * At 8000 Hz that is 10 notifications/second, well within BLE bandwidth.
 * 400 bytes fits in one ATT packet at the MTU=517 the Flutter app negotiates.
 *
 * ── PREREQUISITE prj.conf ─────────────────────────────────────────────
 * CONFIG_NRFX_SAADC=y
 * CONFIG_NRFX_TIMER=y
 * (No per-instance NRFX_TIMERn symbol exists in this NCS version —
 *  the umbrella CONFIG_NRFX_TIMER=y is sufficient for any instance.)
 *
 * ── PREREQUISITE boards/adafruit_feather_nrf52840.overlay ────────────
 * &adc { status = "disabled"; };   // not required (already default-off),
 *                                  // kept for clarity/documentation.
 * (No devicetree entry is required for TIMER2 either — the raw nrfx
 *  driver does not consult devicetree status for instance access.)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
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

/* Use the highest legal (lowest-priority) IRQ priority for this SoC/
 * toolchain config rather than a hardcoded number — the SoftDevice
 * Controller reserves some priority levels for the radio, so the
 * actual ceiling varies. IRQ_PRIO_LOWEST always resolves correctly.
 */
#define SAADC_IRQ_PRIORITY  IRQ_PRIO_LOWEST

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

/* FIX 8: NRFX_TIMER_INSTANCE() needs the peripheral base-address symbol
 * (NRF_TIMER2), not a bare instance number — see fix history above.
 */
static nrfx_timer_t sample_timer = NRFX_TIMER_INSTANCE(NRF_TIMER2);

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
    /* FIX 7: SAADC driver in this nrfx generation returns plain `int`:
     * 0 = success, negative errno = failure. TIMER below still uses the
     * classic nrfx_err_t / NRFX_SUCCESS convention.
     */
    int saadc_err;
    nrfx_err_t timer_err;

    /* ── SAADC channel ───────────────────────────────────────────────── */
    LOG_INF("Configuring SAADC channel");

    nrfx_saadc_channel_t channel =
        NRFX_SAADC_DEFAULT_CHANNEL_SE(PIEZO_AIN_PIN, 0);

    channel.channel_config.gain      = NRF_SAADC_GAIN1_4;
    channel.channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
    channel.channel_config.acq_time  = NRF_SAADC_ACQTIME_10US;

    /* ── SAADC init ──────────────────────────────────────────────────── *
     * FIX 5: manually connect the SAADC IRQ before init. We're bypassing
     * Zephyr's CONFIG_ADC subsystem, so nothing else binds SAADC_IRQn
     * into the vector table for us.
     */
    LOG_INF("SAADC init");

    IRQ_CONNECT(SAADC_IRQn, SAADC_IRQ_PRIORITY, nrfx_isr, nrfx_saadc_irq_handler, 0);

    saadc_err = nrfx_saadc_init(SAADC_IRQ_PRIORITY);
    if (saadc_err != 0) {
        LOG_ERR("SAADC init failed: %d", saadc_err);
        return;
    }
    LOG_INF("SAADC init OK");

    saadc_err = nrfx_saadc_channels_config(&channel, 1);
    if (saadc_err != 0) {
        LOG_ERR("SAADC channel config failed: %d", saadc_err);
        return;
    }

    /* ── FIX 1: advanced mode instead of simple mode ────────────────────
     * Enables external per-sample hardware trigger via the SAMPLE task
     * (PPI-wired from the timer) and START_ON_END for zero-gap capture.
     */
    nrfx_saadc_adv_config_t adv_config =
        NRFX_SAADC_DEFAULT_ADV_CONFIG;

    adv_config.start_on_end = true;

    saadc_err = nrfx_saadc_advanced_mode_set(
        BIT(0),
        NRF_SAADC_RESOLUTION_12BIT,
        &adv_config,
        saadc_handler
    );

    LOG_INF("advanced mode ret=%d", saadc_err);

    if (saadc_err != 0)
    {
        LOG_ERR("SAADC advanced mode failed: %d", saadc_err);
        return;
    }

    /* ── Queue both DMA buffers before starting ──────────────────────── */
    LOG_INF("Queuing DMA buffers");

    saadc_err = nrfx_saadc_buffer_set(saadc_buf_a, BLOCK_SAMPLES);
    if (saadc_err != 0) {
        LOG_ERR("Buffer A queue failed: %d", saadc_err);
        return;
    }

    /* Second call queues buf_b as the "next" buffer; the driver will
     * automatically switch to it when buf_a completes (START_ON_END).
     */
    saadc_err = nrfx_saadc_buffer_set(saadc_buf_b, BLOCK_SAMPLES);
    if (saadc_err != 0) {
        LOG_ERR("Buffer B queue failed: %d", saadc_err);
        return;
    }

    /* ── FIX 2: nrfx_saadc_mode_trigger() instead of nrfx_saadc_start()
     * Arms the driver to accept SAMPLE tasks and points the DMA engine
     * at the first pre-queued buffer. Actual sampling starts when the
     * timer begins firing (below).
     */
    LOG_INF("SAADC mode trigger");
    saadc_err = nrfx_saadc_mode_trigger();
    if (saadc_err != 0) {
        LOG_ERR("SAADC trigger failed: %d", saadc_err);
        return;
    }

    /* ── Hardware timer at SAMPLE_RATE_HZ ────────────────────────────── */
    LOG_INF("Timer init");

    nrfx_timer_config_t timer_cfg = {
        .frequency          = NRF_TIMER_FREQ_1MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_16,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        .p_context          = NULL,
    };

    /* TIMER driver: still the classic nrfx_err_t / NRFX_SUCCESS API. */
    timer_err = nrfx_timer_init(&sample_timer, &timer_cfg, timer_handler);
    if (timer_err != NRFX_SUCCESS) {
        LOG_ERR("Timer init failed: 0x%08x", timer_err);
        return;
    }
    LOG_INF("Timer init OK");

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