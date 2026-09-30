#ifndef DFT_ENGINE_H
#define DFT_ENGINE_H
#define DFT_FRAME_SIZE 1024
#define DFT_NUM_BINS (DFT_FRAME_SIZE / 2)

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t dft_engine_init(void);
esp_err_t dft_engine_process_frame(const int16_t *frame);
esp_err_t dft_engine_get_magnitudes(const float **mags, size_t *num_bins);

void dft_calculate_bin(
   const int16_t *frame,
   float frame_mean,
   size_t k,
   float *real_sum,
   float *imag_sum
);

#endif