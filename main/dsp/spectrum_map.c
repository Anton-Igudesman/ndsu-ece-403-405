#include "spectrum_map.h"
#include <math.h>

esp_err_t spectrum_map_bins_to_bands(
    const float *bins,
    size_t num_bins,
    float *bands,
    size_t num_bands)
{
   if (bins == NULL || bands == NULL) return ESP_ERR_INVALID_ARG;
   if (num_bins == 0 || num_bands == 0) return ESP_ERR_INVALID_ARG;
   if (num_bands > num_bins) return ESP_ERR_INVALID_ARG;

   /*
      Log-spaced bin mapping:
      bins are not 0 indexed to avoid log(0) situations
      edges are uniform in log-space, then projected back to index-space
   */
   const float min_bin_index_f = 1.0f;
   const float max_bin_index_f = (float)num_bins;

   // uniform bands in log-space
   const float log_min = logf(min_bin_index_f);
   const float log_max = logf(max_bin_index_f);
   
   for (size_t band = 0; band < num_bands; band++) 
   {
      // Compute band's L/R edges on a log scale
      const float t0 = (float)band / (float)num_bands;
      const float t1 = (float)(band + 1U) / (float)num_bands;

      size_t start = (size_t)floorf(expf(log_min + t0 * (log_max - log_min)));
      size_t end = (size_t)floorf(expf(log_min + t1 * (log_max - log_min)));

      // Clamp indices into safe bounds
      if (start >= num_bins) start = num_bins - 1U;
      if (end > num_bins) end = num_bins;

      // Ensure non-empty interval [start, end)
      if (end <= start) end = start + 1U;
      if (end > num_bins) end = num_bins;

      float sum = 0.0f;
      for (size_t i = start; i < end; i++) sum += bins[i];

      /* 
         Band mean:
         bands[band] = (1/M) * sum_{i=start}^{end-1}(bins[i])
      */
      bands[band] = sum / (float)(end - start);
   }

   return ESP_OK;
}

// Normalization to help drive LED intensity predictable
esp_err_t spectrum_map_normalize_bands(
   const float *bands_in,
   size_t num_bands,
   float *bands_out)
{
   if (bands_in == NULL || bands_out == NULL || num_bands == 0) return ESP_ERR_INVALID_ARG;

   // Compute max value of the current frame for scaling
   float max_value = 0.0f;
   for (size_t i = 0; i < num_bands; i++) 
   {
      if (bands_in[i] > max_value) max_value = bands_in[i];
   }
   
   // If highest value happens to be 0 - output a zero vector
   if (max_value <= 0.0f)
   {
      for (size_t i = 0; i < num_bands; i++) bands_out[i] = 0.0f;
      return ESP_OK;
   }

   // Per-frame normalization:
   // bands_out[i] = bands_in[i] / max_value
   for (size_t i = 0; i < num_bands; i++) bands_out[i] = bands_in[i] / max_value;
   return ESP_OK;
   
}
