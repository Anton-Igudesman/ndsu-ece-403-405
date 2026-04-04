#ifndef FFT_ENGINE_H
#define FFT_ENGINE_H
#define FFT_FRAME_SIZE 1024
#define FFT_NUM_BINS (FFT_FRAME_SIZE / 2)

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t fft_engine_init(void);
esp_err_t fft_engine_process_frame(const int16_t *frame);
esp_err_t fft_engine_get_magnitudes(const float **mags, size_t *num_bins);

#endif