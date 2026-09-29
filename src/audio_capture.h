#ifndef AUDIO_CAPTURE_H
#define AUDIO_CAPTURE_H

#include <stdint.h>
#include <zephyr/kernel.h>

#define AUDIO_SAMPLE_RATE_HZ  16000
#define AUDIO_BLOCK_SAMPLES   960                        /* 60 ms */
#define AUDIO_BLOCK_BYTES     (AUDIO_BLOCK_SAMPLES * 2)

struct pcm_block {
    int16_t s[AUDIO_BLOCK_SAMPLES];
};

/* Start SAADC + TIMER2 + PPI sampling (mono, 16 kHz). Returns 0 or -errno. */
int audio_capture_init(void);

/* Wait for the next block; DC removal and gain are already applied.
 * Returns 0 on success, -EAGAIN on timeout. */
int audio_capture_get_block(struct pcm_block *blk, k_timeout_t timeout);

#endif /* AUDIO_CAPTURE_H */