/*
 * Synthetic signal generator -- drop-in replacement for the SAADC/piezo
 * audio_capture.c. Produces a test tone (fixed or sweeping) instead of
 * real ADC samples, at the exact same sample rate and block size, so
 * main.c, ble_app.c and ble_audio.c need ZERO changes. This lets you
 * verify the whole BLE audio path with just the nRF52840 dev board --
 * no piezo, no PDM mic, no SD card, nothing wired up.
 *
 * Swap this file in for the real audio_capture.c, build, flash. When
 * you're ready to move to real audio, swap the original file back in;
 * everything else in the project is untouched either way.
 *
 * Tone generation uses a phase-accumulator (DDS): a 32-bit phase value
 * wraps naturally at 2^32, advancing by a fixed step each sample. This
 * gives an exact, driftless output frequency regardless of block timing.
 * A frequency sweep only changes the STEP SIZE each block -- phase itself
 * keeps accumulating continuously, so the tone glides with no clicks or
 * phase jumps at the moment the frequency changes.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(audio_capture, LOG_LEVEL_INF);

/* ---- Change these to test different signals -------------------------- */
#define TONE_AMPLITUDE      8000       /* peak amplitude, 0..32767 */
#define TONE_WAVEFORM       WAVE_SINE  /* WAVE_SINE / WAVE_SQUARE / WAVE_TRIANGLE / WAVE_SAWTOOTH */

/* Set to 0 to go back to a single fixed tone at TONE_FIXED_FREQUENCY_HZ.
 * Set to 1 to sweep continuously between TONE_SWEEP_MIN_HZ and
 * TONE_SWEEP_MAX_HZ -- useful for exercising the whole audio chain (and
 * for watching the oscilloscope timebase/scale on the phone app actually
 * do something, since the number of cycles per division keeps changing). */
#define TONE_SWEEP_ENABLE    1

#define TONE_FIXED_FREQUENCY_HZ  1000U   /* used only when sweep is off */

/* Sweep bounds. Keep TONE_SWEEP_MAX_HZ comfortably under
 * AUDIO_SAMPLE_RATE_HZ/2 (Nyquist, 8000 Hz at 16 kHz sampling) -- 6 kHz
 * leaves headroom and still sounds/looks clean on the sine table. */
#define TONE_SWEEP_MIN_HZ     200U
#define TONE_SWEEP_MAX_HZ     6000U

/* Time for one leg of the sweep (low->high, then high->low takes the
 * same time again, so a full up-down cycle is 2x this). Shorter =
 * faster sweep. */
#define TONE_SWEEP_PERIOD_MS  6000U
/* ------------------------------------------------------------------------ */

typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH,
} waveform_t;

#define SINE_TABLE_BITS 8U
#define SINE_TABLE_SIZE (1U << SINE_TABLE_BITS)

/* sin(2*pi*i/256) scaled to +/-32767, i = 0..255 */
static const int16_t sine_table[SINE_TABLE_SIZE] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602,
    6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793,
    12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530,
    18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
    23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
    30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971,
    32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
    32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285,
    32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683,
    27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
    23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868,
    18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
    12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179,
    6393, 5602, 4808, 4011, 3212, 2410, 1608, 804,
    0, -804, -1608, -2410, -3212, -4011, -4808, -5602,
    -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
    -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
    -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179,
    -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804,
};

static uint32_t phase;
static uint32_t phase_step;

static uint32_t freq_to_phase_step(uint32_t freq_hz)
{
    return (uint32_t)(((uint64_t)freq_hz << 32) / AUDIO_SAMPLE_RATE_HZ);
}

#if TONE_SWEEP_ENABLE
/* Triangle sweep: linearly up from MIN to MAX over one period, then
 * linearly back down over the next period, repeating forever. */
static uint32_t sweep_current_freq_hz(void)
{
    const uint32_t span   = TONE_SWEEP_MAX_HZ - TONE_SWEEP_MIN_HZ;
    const uint32_t cycle  = TONE_SWEEP_PERIOD_MS * 2U;
    uint32_t t = (uint32_t)(k_uptime_get() % cycle);

    if (t < TONE_SWEEP_PERIOD_MS) {
        return TONE_SWEEP_MIN_HZ + (uint32_t)(((uint64_t)span * t) / TONE_SWEEP_PERIOD_MS);
    }

    t -= TONE_SWEEP_PERIOD_MS;
    return TONE_SWEEP_MAX_HZ - (uint32_t)(((uint64_t)span * t) / TONE_SWEEP_PERIOD_MS);
}
#endif

static int16_t next_sample(void)
{
    int16_t sample;

    switch (TONE_WAVEFORM) {
    case WAVE_SQUARE:
        sample = (phase < 0x80000000U) ? TONE_AMPLITUDE : (int16_t)(-TONE_AMPLITUDE);
        break;

    case WAVE_SAWTOOTH: {
        int64_t scaled = ((int64_t)phase * 2 * TONE_AMPLITUDE) >> 32;

        sample = (int16_t)(scaled - TONE_AMPLITUDE);
        break;
    }

    case WAVE_TRIANGLE: {
        bool second_half = phase >= 0x80000000U;
        uint32_t half_phase = second_half ? (phase - 0x80000000U) : phase;
        int64_t scaled = ((int64_t)half_phase * 4 * TONE_AMPLITUDE) >> 32;
        int32_t value = (int32_t)scaled - TONE_AMPLITUDE;

        sample = (int16_t)(second_half ? -value : value);
        break;
    }

    case WAVE_SINE:
    default: {
        uint8_t index = (uint8_t)(phase >> (32U - SINE_TABLE_BITS));

        sample = (int16_t)(((int32_t)sine_table[index] * TONE_AMPLITUDE) / 32767);
        break;
    }
    }

    phase += phase_step;
    return sample;
}

/* Ping-pong buffers, same idea as the real driver's saadc_buf[2]: the
 * timer fills one while the consumer thread still holds the other. */
static int16_t gen_buf[2][AUDIO_BLOCK_SAMPLES];
static uint8_t fill_idx;

K_SEM_DEFINE(block_sem, 0, 1);
static int16_t * volatile ready_buf;
static volatile uint32_t overrun_count;

static void gen_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

#if TONE_SWEEP_ENABLE
    /* Recompute the step size once per block (every 60 ms). Phase itself
     * is untouched, so the waveform stays continuous through the change
     * -- this is standard DDS chirp generation, not a hack. */
    phase_step = freq_to_phase_step(sweep_current_freq_hz());
#endif

    int16_t *buf = gen_buf[fill_idx];

    /* 960 lookups + a multiply/divide each -- a few microseconds total,
     * fine to run directly in this timer's (system clock ISR) context. */
    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        buf[i] = next_sample();
    }

    if (k_sem_count_get(&block_sem) != 0) {
        overrun_count++; /* previous block wasn't consumed in time */
    }

    ready_buf = buf;
    fill_idx ^= 1U;
    k_sem_give(&block_sem);
}

K_TIMER_DEFINE(gen_timer, gen_timer_handler, NULL);

int audio_capture_init(void)
{
    phase = 0U;

#if TONE_SWEEP_ENABLE
    phase_step = freq_to_phase_step(TONE_SWEEP_MIN_HZ);
    LOG_INF("Synthetic signal running: sweeping %u-%u Hz over %u ms/leg, "
            "%d Hz sample rate, block %d samples (%d bytes)",
            TONE_SWEEP_MIN_HZ, TONE_SWEEP_MAX_HZ, TONE_SWEEP_PERIOD_MS,
            AUDIO_SAMPLE_RATE_HZ, AUDIO_BLOCK_SAMPLES, AUDIO_BLOCK_BYTES);
#else
    phase_step = freq_to_phase_step(TONE_FIXED_FREQUENCY_HZ);
    LOG_INF("Synthetic signal running: %u Hz fixed tone, %d Hz sample rate, "
            "block %d samples (%d bytes)",
            TONE_FIXED_FREQUENCY_HZ, AUDIO_SAMPLE_RATE_HZ, AUDIO_BLOCK_SAMPLES,
            AUDIO_BLOCK_BYTES);
#endif

    /* Same cadence the real ADC capture delivered blocks at. */
    k_timer_start(&gen_timer, K_NO_WAIT,
                 K_MSEC(1000ULL * AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_HZ));

    return 0;
}

int audio_capture_get_block(struct pcm_block *blk, k_timeout_t timeout)
{
    static uint32_t block_count;

    if (k_sem_take(&block_sem, timeout) != 0) {
        return -EAGAIN;
    }

    memcpy(blk->s, ready_buf, AUDIO_BLOCK_BYTES);

    if ((++block_count % 16U) == 0U) { /* about once per second */
        int16_t min_v = blk->s[0], max_v = blk->s[0];

        for (int i = 1; i < AUDIO_BLOCK_SAMPLES; i++) {
            if (blk->s[i] < min_v) min_v = blk->s[i];
            if (blk->s[i] > max_v) max_v = blk->s[i];
        }
#if TONE_SWEEP_ENABLE
        LOG_INF("Synthetic block min=%d max=%d freq=%u Hz overruns=%u",
                min_v, max_v, sweep_current_freq_hz(), overrun_count);
#else
        LOG_INF("Synthetic block min=%d max=%d overruns=%u",
                min_v, max_v, overrun_count);
#endif
    }

    return 0;
}