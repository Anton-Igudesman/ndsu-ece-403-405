#include "self_test.h"
#include "common/app_log.h"

#include <math.h>
#include <stdint.h>
#include <driver/i2s_std.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dsp/audio_buffer.h"
#include "dsp/fft_engine.h"
#include "dsp/spectrum_map.h"
#include "common/math_constants.h"

static const char *LOG_TAG = "self_test";
static i2s_chan_handle_t s_test_i2s_rx_chan = NULL;

esp_err_t self_test_log_eq_columns_text(
   const float *bands_norm,
   size_t num_bands,
   uint8_t matrix_height)
{
   // Validate test input contract before formatting.
   if (bands_norm == NULL || 
      num_bands == 0 || 
      matrix_height == 0) return ESP_ERR_INVALID_ARG;
   
   // Single-line text renderer for 8x8-style column view.
   // Example: eq 0:[##......] 1:[####....] ...
   char line[192];
   int write_pos = 0;

   write_pos += snprintf(line + write_pos, sizeof(line) - write_pos, "eq ");

   for (size_t band_index = 0; band_index < num_bands; band_index++)
   {
      // Clamp normalized value so text height mapping stays in [0, matrix_height].
      float normalized_band_level = bands_norm[band_index];
      if (normalized_band_level < 0.0f) normalized_band_level = 0.0f;
      if (normalized_band_level > 1.0f) normalized_band_level = 1.0f;

      uint8_t column_height = (uint8_t)(normalized_band_level * (float)matrix_height + 0.5f);
      if (column_height > matrix_height) column_height = matrix_height;

      // Prefix each column with its index so mapping is obvious in logs.
      write_pos += snprintf(
         line + write_pos,
         sizeof(line) - write_pos,
         "%u:[",
         (unsigned)band_index
      );

      // '#' = active cell, '.' = inactive cell.
      for (uint8_t row_from_bottom = 0; row_from_bottom < matrix_height; row_from_bottom++)
      {
         char cell_char = (row_from_bottom < column_height) ? '#' : '.';
         write_pos += snprintf(line + write_pos, sizeof(line) - write_pos, "%c", cell_char);
      }

      write_pos += snprintf(line + write_pos, sizeof(line) - write_pos, "] ");

      // Stop safely if line buffer is nearly full.
      if (write_pos >= (int)sizeof(line) - 1) break;
   }

   ESP_LOGI("self_test", "%s", line);
   return ESP_OK;
}


// Test creation of frame out of audio samples
static esp_err_t audio_buffer_self_test(void)
{
   audio_buffer_init();

   // Push audio samples to complete a frame
   for (size_t i = 0; i < AUDIO_FRAME_SIZE; i++)
   {
      // Two-tone test signal
      float x1 = 1200.0f * sinf((MATH_TWO_PI * 4.0f * (float)i) / (float)AUDIO_FRAME_SIZE);
      float x2 = 700.0f  * sinf((MATH_TWO_PI * 20.0f * (float)i) / (float)AUDIO_FRAME_SIZE);
      int16_t sample = (int16_t)(x1 + x2);

      audio_buffer_push_sample(sample);
   }

   const int16_t *frame = NULL; // Will be pointer 
   
   // Create full frame from audio samples
   esp_err_t status = audio_buffer_try_get_frame(&frame);
   app_log_error(LOG_TAG, "audio_buffer_try_get_frame", status);
   if (status != ESP_OK || frame == NULL) return ESP_ERR_INVALID_STATE;
   
   ESP_LOGI(LOG_TAG, "audio_buffer self-test OK: n=%u first=%d last=%d",
      (unsigned)AUDIO_FRAME_SIZE,
      frame[0],
      frame[AUDIO_FRAME_SIZE - 1]);
   
   /*
      1) Init FFT engine
      2) Process a frame
      3) Get magnitudes from bins
   */
   status = fft_engine_init();
   app_log_error(LOG_TAG, "fft_engine_init", status); // Status of fft_ingine_init
   if (status != ESP_OK) return status;

   status = fft_engine_process_frame(frame); // process frame
   app_log_error(LOG_TAG, "fft_engine_process_frame", status);
   if (status != ESP_OK) return status;

   size_t bins = 0;
   const float *mags = NULL;
   status = fft_engine_get_magnitudes(&mags, &bins);
   app_log_error(LOG_TAG, "fft_engine_get_magnitudes", status);
   if (status != ESP_OK || mags == NULL || bins == 0) return ESP_ERR_INVALID_STATE;
   
   float mag0 = mags[0];
   float mag1 = (bins > 1) ? mags[1] : 0.0f;

   ESP_LOGI(LOG_TAG, "fft placeholder OK: bins=%u mag0=%.1f mag1=%.1f",
      (unsigned)bins,
      mag0,
      mag1);

   // Map bins into 8 frequency bands for LED mapping
   float bands[SPECTRUM_NUM_BANDS] = {0}; // 8-band output buffer

   status = spectrum_map_bins_to_bands(
      mags,
      bins,
      bands,
      SPECTRUM_NUM_BANDS
   );
   app_log_error(LOG_TAG, "spectrum_map_bins_to_bands", status);
   if (status != ESP_OK) return status;

   ESP_LOGI(LOG_TAG, "spectrum bands: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
      bands[0], bands[1], bands[2], bands[3],
      bands[4], bands[5], bands[6], bands[7]);

   // Normalizing bands for mapping to LED values
   float bands_norm[SPECTRUM_NUM_BANDS] = {0};

   status = spectrum_map_normalize_bands(
      bands, // in
      SPECTRUM_NUM_BANDS,
      bands_norm // out
   );

   app_log_error(LOG_TAG, "spectrum_map_normalize_bands", status);
   if (status != ESP_OK) return status;

   ESP_LOGI(LOG_TAG, "spectrum bands norm: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      bands_norm[0], bands_norm[1], bands_norm[2], bands_norm[3],
      bands_norm[4], bands_norm[5], bands_norm[6], bands_norm[7]);

   return ESP_OK;
}

static void mic_monitor_task(void *arg)
{
   (void)arg;

   int32_t raw_samples[256];
   size_t bytes_read = 0;

   // --- Calibration settings ---
   const int calibration_frames = 30;      // ~3 seconds at 100 ms loop
   const float alpha = 0.10f;              // EMA smoothing factor
   const float voice_ratio_threshold = 2.0f;

   int frame_count = 0;
   float baseline_acc = 0.0f;
   float baseline = 0.0f;
   float ema = 0.0f;
   bool calibrated = false;

   ESP_LOGI(LOG_TAG, "mic_monitor: stay QUIET for calibration...");

   while (true)
   {
      esp_err_t status = i2s_channel_read(
         s_test_i2s_rx_chan,
         raw_samples,
         sizeof(raw_samples),
         &bytes_read,
         pdMS_TO_TICKS(1000)
      );

      if (status != ESP_OK)
      {
         app_log_error(LOG_TAG, "i2s_channel_read", status);
         vTaskDelay(pdMS_TO_TICKS(100));
         continue;
      }

      size_t sample_count = bytes_read / sizeof(int32_t);
      if (sample_count == 0)
      {
         vTaskDelay(pdMS_TO_TICKS(100));
         continue;
      }

      int64_t mean_abs_acc = 0;
      int32_t peak = 0;

      for (size_t i = 0; i < sample_count; i++)
      {
         // Convert 32-bit I2S word to signed 24-bit sample.
         int32_t sample = raw_samples[i] >> 8;
         sample = (sample << 8) >> 8; // explicit sign extension for 24-bit

         int32_t abs_sample = (sample < 0) ? -sample : sample;

         if (abs_sample > peak) peak = abs_sample;

         mean_abs_acc += abs_sample;
      }

      float mean_abs = (float)mean_abs_acc / (float)sample_count;

      // EMA smoothing for more stable level readout.
      if (frame_count == 0) ema = mean_abs;
      else ema = (1.0f - alpha) * ema + alpha * mean_abs;
      
      if (!calibrated)
      {
         baseline_acc += ema;
         frame_count++;

         ESP_LOGI(
            LOG_TAG,
            "calibrating... frame=%d/%d mean_abs=%.1f ema=%.1f",
            frame_count,
            calibration_frames,
            mean_abs,
            ema
         );

         if (frame_count >= calibration_frames)
         {
            baseline = baseline_acc / (float)calibration_frames;
            
            // guard against divide-by-near-zero
            if (baseline < 1.0f) baseline = 1.0f; 
            
            calibrated = true;
            ESP_LOGI(LOG_TAG, "calibration complete: baseline=%.1f", baseline);
            ESP_LOGI(LOG_TAG, "now make noise/speak near the mic");
         }

         vTaskDelay(pdMS_TO_TICKS(100));
         continue;
      }

      float ratio = ema / baseline;
      const char *state = (ratio >= voice_ratio_threshold) ? "VOICE" : "QUIET";

      ESP_LOGI(
         LOG_TAG,
         "mic level: mean_abs=%.1f ema=%.1f baseline=%.1f ratio=%.2f state=%s peak=%ld",
         mean_abs,
         ema,
         baseline,
         ratio,
         state,
         (long)peak
      );

      vTaskDelay(pdMS_TO_TICKS(100));
   }
}

esp_err_t self_test_start_mic_monitor(i2s_chan_handle_t rx_chan)
{
   if (rx_chan == NULL) return ESP_ERR_INVALID_ARG;

   s_test_i2s_rx_chan = rx_chan;

   if (xTaskCreate(mic_monitor_task, "mic_monitor_task", 4096, NULL, 5, NULL) != pdPASS)
   {
      return ESP_FAIL;
   }

   return ESP_OK;
}

// Future target for testing suite
esp_err_t self_test_run_all(void)
{
   return audio_buffer_self_test();
}
