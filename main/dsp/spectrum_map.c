#include "spectrum_map.h"
#include <math.h>
#include "esp_log.h"

#define SPECTRUM_MAP_TRACE_LOGS 0
#define SPECTRUM_MAP_TRACE_DIVISOR 32U

static const char *LOG_TAG = "spectrum_map";
static uint32_t s_spectrum_map_trace_counter = 0;

// ----------------------------------------
// ----- Private Function Definitions -----
// ----------------------------------------

// Project one logarithmic band edge into FFT bin index space
static size_t calculate_log_bin_edge(
   size_t edge,
   size_t num_bands,
   float log_max
)
{
   const float normalized_edge =
      (float)edge / (float)num_bands;

   return (size_t)floorf(
      expf(normalized_edge * log_max)
   );
}

// Clamp a calculated band range into valid FFT bin boundaries
static void clamp_band_range(
   size_t num_bins,
   size_t *start,
   size_t *end
)
{
   if (*start >= num_bins) *start = num_bins - 1U;
   if (*end > num_bins) *end = num_bins;

   // Guarantee a non-empty interval using [start, end)
   if (*end <= *start) *end = *start + 1U;
}

// Calculate the mean magnitude of all FFT bins within [start, end)
static float calculate_band_mean(
   const float *bins,
   size_t start,
   size_t end
)
{
   float sum = 0.0f;

   for (size_t i = start; i < end; i++) sum += bins[i];
   
   const size_t width = end - start;

   return sum / (float)width;
}

// Calculate the FFT bin range assigned to one logarithmically spaced frequency band
esp_err_t spectrum_map_get_band_range(
   size_t num_bins,
   size_t num_bands,
   size_t band,
   size_t *start_out,
   size_t *end_out
)
{
   // Validate spectrum dimensions, requested band, and output pointers
   if (num_bins == 0 || num_bands == 0) return ESP_ERR_INVALID_ARG;
   if (num_bands > num_bins) return ESP_ERR_INVALID_ARG;
   if (band >= num_bands) return ESP_ERR_INVALID_ARG;
   if (start_out == NULL || end_out == NULL) return ESP_ERR_INVALID_ARG;

   // Establish upper boundary of logarithmic bin space
   const float log_max = logf((float)num_bins);

   // Calculate the left and right FFT bin edges for this band
   size_t start = calculate_log_bin_edge(
      band,
      num_bands,
      log_max
   );

   size_t end = calculate_log_bin_edge(
      band + 1U,
      num_bands,
      log_max
   );

   // Keep the calculated interval inside valid FFT bin boundaries
   clamp_band_range(
      num_bins,
      &start,
      &end
   );

   // Return the calculated bin interval to the caller
   *start_out = start;
   *end_out = end;

   return ESP_OK;
}

esp_err_t spectrum_map_bins_to_bands(
    const float *bins,
    size_t num_bins,
    float *bands,
    size_t num_bands)
{
   if (bins == NULL || bands == NULL) return ESP_ERR_INVALID_ARG;
   if (num_bins == 0 || num_bands == 0) return ESP_ERR_INVALID_ARG;
   if (num_bands > num_bins) return ESP_ERR_INVALID_ARG;

   bool trace_this_call = false;

#if SPECTRUM_MAP_TRACE_LOGS
   s_spectrum_map_trace_counter++;
   trace_this_call = ((s_spectrum_map_trace_counter % SPECTRUM_MAP_TRACE_DIVISOR) == 0U);
#endif
   
   for (size_t band = 0; band < num_bands; band++) 
   {
      size_t start = 0U;
      size_t end = 0U;

      // Get the logarithmically spaced FFT bin range for this band
      esp_err_t status = spectrum_map_get_band_range(
         num_bins,
         num_bands,
         band,
         &start,
         &end
      );

      if (status != ESP_OK) return status;

      // Average the FFT magnitudes assigned to this band
      bands[band] = calculate_band_mean(
         bins,
         start,
         end
      );

      if (trace_this_call)
      {
         const size_t width = end - start;

         ESP_LOGI(
            LOG_TAG,
            "Band-mapping step (band %u): bin range [%u, %u); width=%u bins; band mean=%.2f",
            (unsigned)band,
            (unsigned)start,
            (unsigned)end,
            (unsigned)width,
            bands[band]
         );
      }
   }

   return ESP_OK;
}
