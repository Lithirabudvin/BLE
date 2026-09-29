/*
 * Piezo capture: TIMER2 (16 MHz) -> PPI -> SAADC SAMPLE task -> EasyDMA
 * ping-pong buffers. Mono, 16 kHz, 12-bit samples in 16-bit containers.
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <nrfx.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_saadc.h>

#include "audio_capture.h"

LOG_MODULE_REGISTER(audio_capture, LOG_LEVEL_INF);

#define PIEZO_AIN_PIN       NRF_SAADC_INPUT_AIN1
#define TIMER_FREQ_HZ       16000000UL
#define SAADC_IRQ_PRIORITY  IRQ_PRIO_LOWEST

/* 12-bit samples are 0..4095; x16 fills the 16-bit range. Tune to taste. */
#define GAIN_FACTOR         16

/* DC blocker time constant: 2^DC_SHIFT samples (10 -> ~64 ms). */
#define DC_SHIFT            10

/* Change if the BLE controller on your build already uses this channel. */
#define SAMPLE_PPI_CHANNEL  NRF_PPI_CHANNEL0

static int16_t saadc_buf[2][AUDIO_BLOCK_SAMPLES];
static nrfx_timer_t sample_timer = NRFX_TIMER_INSTANCE(NRF_TIMER2);

K_SEM_DEFINE(block_sem, 0, 1);
static int16_t * volatile ready_buf;
static volatile uint32_t overrun_count;

static int32_t dc_q8;
static bool    dc_valid;

static void center_and_gain(int16_t *buf, size_t count)
{
    if (!dc_valid) {
        dc_q8 = ((int32_t)buf[0]) << 8;
        dc_valid = true;
    }

    for (size_t i = 0; i < count; i++) {
        int32_t x = buf[i];

        dc_q8 += ((x << 8) - dc_q8) >> DC_SHIFT;

        int32_t amplified = (x - (dc_q8 >> 8)) * GAIN_FACTOR;

        if (amplified > 32767)  amplified = 32767;
        if (amplified < -32768) amplified = -32768;

        buf[i] = (int16_t)amplified;
    }
}

static void saadc_handler(nrfx_saadc_evt_t const *event)
{
    static uint8_t next_buf = 1;   /* buffer 0 is queued before start */

    switch (event->type) {
    case NRFX_SAADC_EVT_DONE:
        if (k_sem_count_get(&block_sem) != 0) {
            overrun_count++;
        }
        ready_buf = (int16_t *)event->data.done.p_buffer;
        k_sem_give(&block_sem);
        break;

    case NRFX_SAADC_EVT_BUF_REQ:
        (void)nrfx_saadc_buffer_set(saadc_buf[next_buf], AUDIO_BLOCK_SAMPLES);
        next_buf ^= 1;
        break;

    default:
        break;
    }
}

static void timer_handler(nrf_timer_event_t event_type, void *ctx)
{
    ARG_UNUSED(event_type);
    ARG_UNUSED(ctx);
}

int audio_capture_init(void)
{
    int err;

    nrfx_saadc_channel_t channel =
        NRFX_SAADC_DEFAULT_CHANNEL_SE(PIEZO_AIN_PIN, 0);

    channel.channel_config.gain      = NRF_SAADC_GAIN1_4;
    channel.channel_config.reference = NRF_SAADC_REFERENCE_VDD4;
    channel.channel_config.acq_time  = NRF_SAADC_ACQTIME_10US;

    IRQ_CONNECT(SAADC_IRQn, SAADC_IRQ_PRIORITY, nrfx_isr,
                nrfx_saadc_irq_handler, 0);

    err = nrfx_saadc_init(SAADC_IRQ_PRIORITY);
    if (err != 0) {
        LOG_ERR("SAADC init failed: %d", err);
        return err;
    }

    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != 0) {
        LOG_ERR("SAADC channel config failed: %d", err);
        return err;
    }

    nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    adv_config.start_on_end = true;

    err = nrfx_saadc_advanced_mode_set(BIT(0), NRF_SAADC_RESOLUTION_12BIT,
                                       &adv_config, saadc_handler);
    if (err != 0) {
        LOG_ERR("SAADC advanced mode failed: %d", err);
        return err;
    }

    /* Only buffer 0 here; BUF_REQ supplies buffer 1, then 0, 1, ... */
    err = nrfx_saadc_buffer_set(saadc_buf[0], AUDIO_BLOCK_SAMPLES);
    if (err != 0) {
        LOG_ERR("Buffer queue failed: %d", err);
        return err;
    }

    err = nrfx_saadc_mode_trigger();
    if (err != 0) {
        LOG_ERR("SAADC trigger failed: %d", err);
        return err;
    }

    nrfx_timer_config_t timer_cfg = {
        .frequency          = TIMER_FREQ_HZ,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_16,
        .interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
        .p_context          = NULL,
    };

    err = nrfx_timer_init(&sample_timer, &timer_cfg, timer_handler);
    if (err != 0) {
        LOG_ERR("Timer init failed: %d", err);
        return err;
    }

    nrfx_timer_extended_compare(&sample_timer, NRF_TIMER_CC_CHANNEL0,
                                TIMER_FREQ_HZ / AUDIO_SAMPLE_RATE_HZ,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    nrf_ppi_channel_endpoint_setup(
        NRF_PPI, SAMPLE_PPI_CHANNEL,
        nrfx_timer_compare_event_address_get(&sample_timer,
                                             NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    nrf_ppi_channel_enable(NRF_PPI, SAMPLE_PPI_CHANNEL);
    nrfx_timer_enable(&sample_timer);

    LOG_INF("SAADC running: %d Hz mono, block %d samples (%d bytes)",
            AUDIO_SAMPLE_RATE_HZ, AUDIO_BLOCK_SAMPLES, AUDIO_BLOCK_BYTES);
    return 0;
}

int audio_capture_get_block(struct pcm_block *blk, k_timeout_t timeout)
{
    static uint32_t block_count;

    if (k_sem_take(&block_sem, timeout) != 0) {
        return -EAGAIN;
    }

    /* Copy out of the DMA buffer immediately, then process the copy. */
    memcpy(blk->s, ready_buf, AUDIO_BLOCK_BYTES);

    if ((++block_count % 16U) == 0U) {          /* about once per second */
        int16_t min_v = blk->s[0], max_v = blk->s[0];

        for (int i = 1; i < AUDIO_BLOCK_SAMPLES; i++) {
            if (blk->s[i] < min_v) min_v = blk->s[i];
            if (blk->s[i] > max_v) max_v = blk->s[i];
        }
        LOG_INF("ADC raw min=%d max=%d overruns=%u",
                min_v, max_v, overrun_count);
    }

    center_and_gain(blk->s, AUDIO_BLOCK_SAMPLES);
    return 0;
}