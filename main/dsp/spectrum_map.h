#ifndef SPECTRUM_MAP_H
#define SPECTRUM_MAP_H

#include <stddef.h>
#include "esp_err.h"

#define SPECTRUM_NUM_BANDS 8

esp_err_t spectrum_map_bins_to_bands(
   const float *bins,
   size_t num_bins,
   float *bands,
   size_t num_bands
);

esp_err_t spectrum_map_get_band_range(
   size_t num_bins,
   size_t num_bands,
   size_t band,
   size_t *start_out,
   size_t *end_out
);

#endif