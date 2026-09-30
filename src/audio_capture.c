/*
 * Synthetic bowel-sound generator -- drop-in replacement for the
 * SAADC/piezo audio_capture.c (same API, sample rate and block size), so
 * main.c, ble_app.c and ble_audio.c need no changes.
 *
 * This is a TEST signal for exercising the BLE pipeline and the phone
 * display with something that looks and sounds like abdominal
 * auscultation. It is a rough approximation, not a physiological model.
 *
 * It mixes:
 *   - background: low-level band-limited noise (sensor / body noise floor)
 *   - bowel-sound events at random (Poisson) times, of the four types
 *     commonly used to classify bowel sounds:
 *       SB  single burst      one short click/pop, ~10-30 ms
 *       MB  multiple bursts   a train of 3-10 clicks with short gaps
 *       CRS continuous random sound: dense random clicks for 0.3-2 s
 *       HS  harmonic sound    a short tonal "gurgle" with harmonics
 *   - optional heartbeat thumps (a common interference on the abdomen)
 *
 * Each click is a damped sinusoid from a two-pole resonator
 *     y[n] = 2 r cos(w) y[n-1] - r^2 y[n-2]
 * kicked by a single impulse, which rings as A * r^n * sin(w (n+1)).
 * A small pool of these "voices" lets clicks overlap.
 *
 * REQUIRES THE FPU -- add to prj.conf:
 *     CONFIG_FPU=y
 *     CONFIG_FPU_SHARING=y
 *
 * Samples are generated in audio_capture_get_block() (thread context)
 * instead of the timer ISR, because the floating-point work does not
 * belong in an interrupt. The timer only counts out block periods; if the
 * consumer falls behind by up to MAX_PENDING_BLOCKS blocks it catches up
 * without losing any samples.
 *
 * Works at any AUDIO_SAMPLE_RATE_HZ >= 4000 (8000 is plenty: almost all
 * of this signal's energy is below 1.5 kHz).
 */

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(audio_capture, LOG_LEVEL_INF);

/* ---- Change these to shape the signal ---------------------------------- */

/* Average number of bowel-sound events per minute. Common clinical rules
 * of thumb put normal activity at roughly 5-35 per minute; try ~40+ for
 * "hyperactive" and ~3 for "hypoactive". */
#define EVENTS_PER_MIN        15.0f

/* Relative likelihood of each event type (any non-negative integers). */
#define WEIGHT_SB             45
#define WEIGHT_MB             25
#define WEIGHT_CRS            15
#define WEIGHT_HS             15

/* Overall level multiplier. At 1.0 the loudest clicks peak around half of
 * full scale. */
#define OUTPUT_GAIN           1.0f

/* Background noise floor: RMS as a fraction of full scale, and its upper
 * frequency limit. */
#define BACKGROUND_RMS        0.004f
#define BACKGROUND_CUTOFF_HZ  600.0f

/* Single burst (these frequency/decay ranges are also used for every click
 * of a multiple burst). */
#define SB_FREQ_MIN_HZ        200.0f
#define SB_FREQ_MAX_HZ        700.0f
#define SB_DECAY_MIN_MS       2.0f     /* ring-down time constant */
#define SB_DECAY_MAX_MS       6.0f
#define SB_AMP_MIN            0.15f
#define SB_AMP_MAX            0.50f

/* Multiple bursts. */
#define MB_CLICKS_MIN         3
#define MB_CLICKS_MAX         10
#define MB_GAP_MIN_MS         20.0f
#define MB_GAP_MAX_MS         150.0f
#define MB_AMP_MIN            0.10f
#define MB_AMP_MAX            0.35f

/* Continuous random sound. */
#define CRS_DUR_MIN_MS        300.0f
#define CRS_DUR_MAX_MS        2000.0f
#define CRS_SPACING_MIN_MS    3.0f
#define CRS_SPACING_MAX_MS    15.0f
#define CRS_FREQ_MIN_HZ       150.0f
#define CRS_FREQ_MAX_HZ       900.0f
#define CRS_DECAY_MIN_MS      1.5f
#define CRS_DECAY_MAX_MS      5.0f
#define CRS_AMP_MIN           0.03f
#define CRS_AMP_MAX           0.12f

/* Harmonic sound: fundamental plus 2nd and 3rd harmonics, with a slight
 * pitch glide and a "bubbling" amplitude flutter. */
#define HS_F0_MIN_HZ          150.0f
#define HS_F0_MAX_HZ          400.0f
#define HS_GLIDE              0.15f    /* max +/- pitch change over the sound */
#define HS_DUR_MIN_MS         100.0f
#define HS_DUR_MAX_MS         800.0f
#define HS_AMP_MIN            0.08f
#define HS_AMP_MAX            0.25f

/* Heartbeat thumps (S1 + S2 pairs). 0 = off, 1 = on. */
#define HEART_ENABLE          0
#define HEART_BPM             72.0f
#define HEART_LEVEL           0.03f

/* Same seed = same sequence of events after every reset. */
#define RANDOM_SEED           0x2545F491U

/* ------------------------------------------------------------------------ */

#define FS                    ((float)AUDIO_SAMPLE_RATE_HZ)
#define TWO_PI                6.28318531f
#define PI_F                  3.14159265f
#define NUM_VOICES            12
#define MAX_PENDING_BLOCKS    4

BUILD_ASSERT(AUDIO_SAMPLE_RATE_HZ >= 4000,
             "bowel-sound generator needs at least 4 kHz sampling");
BUILD_ASSERT(WEIGHT_SB + WEIGHT_MB + WEIGHT_CRS + WEIGHT_HS > 0,
             "at least one event type must have a non-zero weight");
BUILD_ASSERT(MB_CLICKS_MIN >= 1 && MB_CLICKS_MIN <= MB_CLICKS_MAX,
             "bad multiple-burst click range");

/* ---- Random numbers ----------------------------------------------------- */

static uint32_t rng_state = RANDOM_SEED;

static uint32_t rand_u32(void)
{
    uint32_t x = rng_state;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

/* Uniform in [0, 1). */
static float rand_unit(void)
{
    return (float)(rand_u32() >> 8) * (1.0f / 16777216.0f);
}

static float rand_range(float lo, float hi)
{
    return lo + (hi - lo) * rand_unit();
}

static int rand_int(int lo, int hi)
{
    return lo + (int)(rand_u32() % (uint32_t)(hi - lo + 1));
}

static uint32_t ms_to_samples(float ms)
{
    return (uint32_t)(ms * FS / 1000.0f);
}

/* ---- Click voices (damped resonators) ---------------------------------- */

struct voice {
    float c1, c2;    /* 2 r cos(w), -r^2 */
    float y1, y2;    /* previous two outputs */
    float kick;      /* impulse applied on the next sample */
    uint32_t age;
    bool active;
};

static struct voice voices[NUM_VOICES];

static void voice_trigger(float freq_hz, float decay_ms, float amp)
{
    /* Take a free voice; if all are busy, steal the quietest one. */
    struct voice *v = &voices[0];
    float quietest = 1e9f;

    for (int i = 0; i < NUM_VOICES; i++) {
        if (!voices[i].active) {
            v = &voices[i];
            break;
        }
        float level = fabsf(voices[i].y1) + fabsf(voices[i].y2);

        if (level < quietest) {
            quietest = level;
            v = &voices[i];
        }
    }

    float w = TWO_PI * freq_hz / FS;
    float r = expf(-1000.0f / (decay_ms * FS));

    v->c1 = 2.0f * r * cosf(w);
    v->c2 = -r * r;
    v->y1 = 0.0f;
    v->y2 = 0.0f;
    v->kick = amp * sinf(w);   /* then rings as amp * r^n * sin(w (n+1)) */
    v->age = 0U;
    v->active = true;
}

static float voices_next(void)
{
    float sum = 0.0f;

    for (int i = 0; i < NUM_VOICES; i++) {
        struct voice *v = &voices[i];

        if (!v->active) {
            continue;
        }

        float y = v->c1 * v->y1 + v->c2 * v->y2 + v->kick;

        v->kick = 0.0f;
        v->y2 = v->y1;
        v->y1 = y;
        sum += y;

        /* Free the voice once it has rung down below ~3 LSB. */
        if (++v->age > 16U && (fabsf(v->y1) + fabsf(v->y2)) < 1e-4f) {
            v->active = false;
        }
    }

    return sum;
}

/* ---- Event scheduling --------------------------------------------------- */

enum seq_type { SEQ_NONE, SEQ_MB, SEQ_CRS };

static enum seq_type seq_state;
static int      seq_clicks_left;
static uint32_t seq_countdown;
static uint32_t seq_pos, seq_len;
static uint32_t idle_countdown;   /* samples until the next event starts */

static volatile uint32_t count_sb, count_mb, count_crs, count_hs;

/* Exponentially distributed gaps give Poisson-distributed event times:
 * irregular, with occasional clusters and occasional long silences. */
static uint32_t event_gap_samples(void)
{
    float mean_s = 60.0f / EVENTS_PER_MIN;
    float u = rand_unit();

    if (u < 1e-4f) {
        u = 1e-4f;
    }

    float gap_s = -mean_s * logf(u);

    if (gap_s < 0.05f) {
        gap_s = 0.05f;
    }

    return (uint32_t)(gap_s * FS);
}

static void event_finished(void)
{
    idle_countdown = event_gap_samples();
}

/* ---- Harmonic sound ----------------------------------------------------- */

static struct {
    bool active;
    float phase, f_start, f_end, amp;
    float am_phase, am_step;
    uint32_t pos, len, ramp;
} hs;

static float hs_next(void)
{
    if (!hs.active) {
        return 0.0f;
    }

    float t = (float)hs.pos / (float)hs.len;
    float f = hs.f_start + (hs.f_end - hs.f_start) * t;

    hs.phase += TWO_PI * f / FS;
    if (hs.phase >= TWO_PI) {
        hs.phase -= TWO_PI;
    }
    hs.am_phase += hs.am_step;
    if (hs.am_phase >= TWO_PI) {
        hs.am_phase -= TWO_PI;
    }

    float p = hs.phase;
    float s = (sinf(p) + 0.5f * sinf(2.0f * p) + 0.25f * sinf(3.0f * p))
              * (1.0f / 1.75f);

    float env = 1.0f;

    if (hs.pos < hs.ramp) {
        env = (float)hs.pos / (float)hs.ramp;
    } else if (hs.len - hs.pos < hs.ramp) {
        env = (float)(hs.len - hs.pos) / (float)hs.ramp;
    }
    env *= 0.75f + 0.25f * sinf(hs.am_phase);   /* bubbling flutter */

    if (++hs.pos >= hs.len) {
        hs.active = false;
        event_finished();
    }

    return hs.amp * env * s;
}

/* ---- Starting events ---------------------------------------------------- */

static void start_event(void)
{
    const int total = WEIGHT_SB + WEIGHT_MB + WEIGHT_CRS + WEIGHT_HS;
    int pick = rand_int(0, total - 1);

    if ((pick -= WEIGHT_SB) < 0) {
        float f = rand_range(SB_FREQ_MIN_HZ, SB_FREQ_MAX_HZ);

        voice_trigger(f, rand_range(SB_DECAY_MIN_MS, SB_DECAY_MAX_MS),
                      rand_range(SB_AMP_MIN, SB_AMP_MAX));
        count_sb++;
        LOG_INF("Event SB: %d Hz", (int)f);
        event_finished();

    } else if ((pick -= WEIGHT_MB) < 0) {
        seq_state = SEQ_MB;
        seq_clicks_left = rand_int(MB_CLICKS_MIN, MB_CLICKS_MAX);
        seq_countdown = 0U;
        count_mb++;
        LOG_INF("Event MB: %d clicks", seq_clicks_left);

    } else if ((pick -= WEIGHT_CRS) < 0) {
        seq_state = SEQ_CRS;
        seq_pos = 0U;
        seq_len = ms_to_samples(rand_range(CRS_DUR_MIN_MS, CRS_DUR_MAX_MS));
        seq_countdown = 0U;
        count_crs++;
        LOG_INF("Event CRS: %d ms", (int)(seq_len * 1000U / AUDIO_SAMPLE_RATE_HZ));

    } else {
        hs.f_start = rand_range(HS_F0_MIN_HZ, HS_F0_MAX_HZ);
        hs.f_end = hs.f_start * (1.0f + rand_range(-HS_GLIDE, HS_GLIDE));
        hs.amp = rand_range(HS_AMP_MIN, HS_AMP_MAX);
        hs.len = ms_to_samples(rand_range(HS_DUR_MIN_MS, HS_DUR_MAX_MS));
        hs.ramp = MIN(ms_to_samples(20.0f), hs.len / 4U);
        hs.pos = 0U;
        hs.phase = 0.0f;
        hs.am_phase = 0.0f;
        hs.am_step = TWO_PI * rand_range(6.0f, 15.0f) / FS;
        hs.active = true;
        count_hs++;
        LOG_INF("Event HS: %d Hz, %d ms", (int)hs.f_start,
                (int)(hs.len * 1000U / AUDIO_SAMPLE_RATE_HZ));
    }
}

static void scheduler_tick(void)
{
    switch (seq_state) {
    case SEQ_MB:
        if (seq_countdown > 0U) {
            seq_countdown--;
            break;
        }
        voice_trigger(rand_range(SB_FREQ_MIN_HZ, SB_FREQ_MAX_HZ),
                      rand_range(SB_DECAY_MIN_MS, SB_DECAY_MAX_MS),
                      rand_range(MB_AMP_MIN, MB_AMP_MAX));
        if (--seq_clicks_left <= 0) {
            seq_state = SEQ_NONE;
            event_finished();
        } else {
            seq_countdown = ms_to_samples(rand_range(MB_GAP_MIN_MS, MB_GAP_MAX_MS));
        }
        break;

    case SEQ_CRS:
        if (seq_countdown > 0U) {
            seq_countdown--;
        } else {
            /* Smooth rise and fall over the whole sound. */
            float env = sinf(PI_F * (float)seq_pos / (float)seq_len);

            voice_trigger(rand_range(CRS_FREQ_MIN_HZ, CRS_FREQ_MAX_HZ),
                          rand_range(CRS_DECAY_MIN_MS, CRS_DECAY_MAX_MS),
                          env * rand_range(CRS_AMP_MIN, CRS_AMP_MAX));
            seq_countdown = ms_to_samples(rand_range(CRS_SPACING_MIN_MS,
                                                     CRS_SPACING_MAX_MS));
        }
        if (++seq_pos >= seq_len) {
            seq_state = SEQ_NONE;
            event_finished();
        }
        break;

    case SEQ_NONE:
    default:
        if (hs.active) {
            break;              /* a harmonic sound is still playing */
        }
        if (idle_countdown > 0U) {
            idle_countdown--;
        } else {
            start_event();
        }
        break;
    }
}

/* ---- Heartbeat ---------------------------------------------------------- */

#if HEART_ENABLE
static uint32_t heart_countdown;
static uint32_t heart_s2_countdown;

static void heart_tick(void)
{
    if (heart_countdown == 0U) {
        voice_trigger(rand_range(40.0f, 60.0f), 30.0f, HEART_LEVEL);          /* S1 */
        heart_s2_countdown = ms_to_samples(300.0f);
        heart_countdown = (uint32_t)(60.0f / HEART_BPM * FS);
    } else {
        heart_countdown--;
    }

    if (heart_s2_countdown > 0U && --heart_s2_countdown == 0U) {
        voice_trigger(rand_range(50.0f, 70.0f), 20.0f, HEART_LEVEL * 0.7f);   /* S2 */
    }
}
#endif

/* ---- Background noise --------------------------------------------------- */

static float bg_state, bg_alpha, bg_gain;

static float background_next(void)
{
    float n = rand_unit() * 2.0f - 1.0f;

    bg_state += bg_alpha * (n - bg_state);   /* one-pole low-pass */
    return bg_state * bg_gain;
}

/* ---- Sample output ------------------------------------------------------ */

static volatile uint32_t clip_count;

static int16_t next_sample(void)
{
    scheduler_tick();
#if HEART_ENABLE
    heart_tick();
#endif

    float x = (voices_next() + hs_next() + background_next()) * OUTPUT_GAIN;
    int32_t s = (int32_t)(x * 32767.0f);

    if (s > 32767) {
        s = 32767;
        clip_count++;
    } else if (s < -32768) {
        s = -32768;
        clip_count++;
    }

    return (int16_t)s;
}

/* ---- Block timing --------------------------------------------------------- */

/* The timer only counts block periods; samples are made in thread context. */
K_SEM_DEFINE(block_sem, 0, MAX_PENDING_BLOCKS);
static volatile uint32_t overrun_count;

static void gen_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    if (k_sem_count_get(&block_sem) >= MAX_PENDING_BLOCKS) {
        overrun_count++;    /* consumer is more than MAX_PENDING_BLOCKS behind */
    }
    k_sem_give(&block_sem);
}

K_TIMER_DEFINE(gen_timer, gen_timer_handler, NULL);

int audio_capture_init(void)
{
    /* Normalise the background so its RMS equals BACKGROUND_RMS:
     * uniform noise has RMS 1/sqrt(3), and a one-pole low-pass scales it
     * by sqrt(a / (2 - a)). */
    bg_alpha = 1.0f - expf(-TWO_PI * BACKGROUND_CUTOFF_HZ / FS);
    bg_gain = BACKGROUND_RMS / (0.57735f * sqrtf(bg_alpha / (2.0f - bg_alpha)));
    bg_state = 0.0f;

    seq_state = SEQ_NONE;
    hs.active = false;
    idle_countdown = event_gap_samples();

    LOG_INF("Synthetic bowel sounds: ~%d events/min (SB %d, MB %d, CRS %d, HS %d), "
            "heart %s, %d Hz sample rate, block %d samples",
            (int)EVENTS_PER_MIN, WEIGHT_SB, WEIGHT_MB, WEIGHT_CRS, WEIGHT_HS,
            HEART_ENABLE ? "on" : "off", AUDIO_SAMPLE_RATE_HZ, AUDIO_BLOCK_SAMPLES);

    k_timer_start(&gen_timer, K_NO_WAIT,
                  K_MSEC(1000ULL * AUDIO_BLOCK_SAMPLES / AUDIO_SAMPLE_RATE_HZ));

    return 0;
}

int audio_capture_get_block(struct pcm_block *blk, k_timeout_t timeout)
{
    static uint32_t block_count;
    static int16_t window_min = INT16_MAX;
    static int16_t window_max = INT16_MIN;

    if (k_sem_take(&block_sem, timeout) != 0) {
        return -EAGAIN;
    }

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        int16_t s = next_sample();

        blk->s[i] = s;
        if (s < window_min) {
            window_min = s;
        }
        if (s > window_max) {
            window_max = s;
        }
    }

    /* Summary about once per second (at 16 kHz; every ~2 s at 8 kHz). */
    if ((++block_count % 16U) == 0U) {
        LOG_INF("Bowel sim: min=%d max=%d | SB=%u MB=%u CRS=%u HS=%u | "
                "clipped=%u overruns=%u",
                window_min, window_max, count_sb, count_mb, count_crs,
                count_hs, clip_count, overrun_count);
        window_min = INT16_MAX;
        window_max = INT16_MIN;
    }

    return 0;
}