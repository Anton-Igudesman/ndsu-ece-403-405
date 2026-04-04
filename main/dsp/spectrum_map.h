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

esp_err_t spectrum_map_normalize_bands(
   const float *bands_in,
   size_t num_bands,
   float *bands_out
);

#endif