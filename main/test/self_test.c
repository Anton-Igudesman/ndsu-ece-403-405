#include "self_test.h"
#include "common/app_log.h"

#include <math.h>
#include <stdint.h>
#include <driver/i2s_std.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dsp/audio_buffer.h"
#include "dsp/dft_engine.h"
#include "dsp/fft_engine.h"
#include "dsp/spectrum_map.h"
#include "led/led_cube_mapper.h"
#include "common/math_constants.h"

typedef struct
{
   size_t x;
   size_t y;
   size_t z;
   size_t expected_layer;
   size_t expected_pixel_index;
} cube_mapper_test_case_t;


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

typedef esp_err_t (*transform_process_fn_t)(const int16_t *frame);

static esp_err_t run_timed_transform(
   const char *transform_name,
   transform_process_fn_t process_fn,
   const int16_t *frame
)
{
   if (transform_name == NULL ||
      process_fn == NULL ||
      frame == NULL) return ESP_ERR_INVALID_ARG;
   
   int64_t start_us = esp_timer_get_time();

   // Callback
   esp_err_t status = process_fn(frame);

   int64_t elapsed_us = esp_timer_get_time() - start_us;

   app_log_error(LOG_TAG, transform_name, status);
   if (status != ESP_OK) return status;

   ESP_LOGI(
      LOG_TAG,
      "%s process time: %lld us",
      transform_name,
      (long long)elapsed_us
   );

   return ESP_OK;
}

static esp_err_t run_dft_test(
   const int16_t *frame,
   const float **mags_out,
   size_t *bins_out
)
{
   /*
      1) Init DFT engine
      2) Process a frame
      3) Get magnitudes from bins
   */

   if (frame == NULL || mags_out == NULL || bins_out == NULL) return ESP_ERR_INVALID_ARG;

   esp_err_t status = dft_engine_init();
   app_log_error(LOG_TAG, "dft_engine_init", status); // Status of dft_ingine_init
   if (status != ESP_OK) return status;

   status = run_timed_transform(
      "DFT",
      dft_engine_process_frame,
      frame
   );
   if (status != ESP_OK) return status;
   
   status = dft_engine_get_magnitudes(mags_out, bins_out);
   app_log_error(LOG_TAG, "dft_engine_get_magnitudes", status);
   if (status != ESP_OK) return status; 
   if (*mags_out == NULL || *bins_out == 0) return ESP_ERR_INVALID_STATE;

   return ESP_OK;
}

static esp_err_t run_fft_test(
   const int16_t *frame,
   const float **mags_out,
   size_t *bins_out
)
{
   if (frame == NULL || mags_out == NULL || bins_out == NULL) return ESP_ERR_INVALID_ARG;

   esp_err_t status = fft_engine_init();
   app_log_error(LOG_TAG, "fft_engine_init", status);
   if (status != ESP_OK) return status;

   status = run_timed_transform(
      "FFT",
      fft_engine_process_frame,
      frame
   );
   if (status != ESP_OK) return status;

   status = fft_engine_get_magnitudes(mags_out, bins_out);
   app_log_error(LOG_TAG, "fft_engine_get_magnitudes", status);
   if (status != ESP_OK) return status;
   if (*mags_out == NULL || *bins_out == 0) return ESP_ERR_INVALID_STATE;

   return ESP_OK;
}

static esp_err_t generate_test_audio_frame(
   const size_t *test_bins,
   const float *test_amplitudes,
   size_t test_tone_count,
   const int16_t **frame_out
)
{
   if (test_bins == NULL ||
      test_amplitudes == NULL ||
      test_tone_count == 0 ||
      frame_out == NULL) return ESP_ERR_INVALID_ARG;

   for (size_t i = 0; i < AUDIO_FRAME_SIZE; i++)
   {
      float sample_sum = 0.0f;

      // Building test signal
      for (size_t tone = 0; tone < test_tone_count; tone++)
      {
         sample_sum += test_amplitudes[tone] * sinf(
            (MATH_TWO_PI * (float)test_bins[tone] * (float)i) /
         (float)AUDIO_FRAME_SIZE
         );
      }

      int16_t sample = (int16_t)sample_sum;
      audio_buffer_push_sample(sample);
   } 
   
   // Create full frame from audio samples
   esp_err_t status = audio_buffer_try_get_frame(frame_out);
   app_log_error(LOG_TAG, "audio_buffer_try_get_frame", status);
   if (status != ESP_OK) return status;
   if (*frame_out == NULL) return ESP_ERR_INVALID_STATE;
   
   ESP_LOGI(LOG_TAG, "audio_buffer self-test OK: n=%u first=%d last=%d",
      (unsigned)AUDIO_FRAME_SIZE,
      (*frame_out)[0],
      (*frame_out)[AUDIO_FRAME_SIZE - 1]);

   return ESP_OK;
}

static esp_err_t test_spectrum_mapping(
   const float *mags,
   size_t bins
)
{
   if (mags == NULL || bins == 0) return ESP_ERR_INVALID_ARG;

   // Map bins into 8 frequency bands for LED mapping
   float bands[SPECTRUM_NUM_BANDS] = {0}; // 8-band output buffer

   esp_err_t status = spectrum_map_bins_to_bands(
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

static void log_transform_test_result(
   const char *transform_name,
   const float *mags,
   size_t bins
)
{
   if (transform_name == NULL || mags == NULL || bins == 0) return;

   float mag0 = mags[0];
   float mag1 = (bins > 1) ? mags[1] : 0.0f;

   ESP_LOGI(
      LOG_TAG, 
      "%s test OK: bins=%u mag0=%.1f mag1=%.1f",
      transform_name,
      (unsigned)bins,
      mag0,
      mag1);
}

static esp_err_t compare_dft_fft(
   const float *dft_mags,
   size_t dft_bins,
   const float *fft_mags,
   size_t fft_bins
)
{
   if (dft_mags == NULL || fft_mags == NULL) return ESP_ERR_INVALID_ARG;
   if (dft_bins == 0 || fft_bins == 0) return ESP_ERR_INVALID_ARG;

   // Bin count check
   if (dft_bins != fft_bins)
   {
      ESP_LOGE(
         LOG_TAG,
         "DFT/FFT bin count mismatch: DFT=%u FFT=%u",
         (unsigned)dft_bins,
         (unsigned)fft_bins
      );

      return ESP_FAIL;
   }

   float max_abs_diff = 0.0f;
   size_t max_diff_bin = 0;

   // Loop through bins and remember the WORST disagreement
   for (size_t k = 0; k < dft_bins; k++)
   {
      float abs_diff = fabsf(dft_mags[k] - fft_mags[k]);

      if (abs_diff > max_abs_diff)
      {
         max_abs_diff = abs_diff;
         max_diff_bin = k;
      }
   }

   float dft_value = dft_mags[max_diff_bin];
   float fft_value = fft_mags[max_diff_bin];

   float relative_error = 0.0f;
   if (fabsf(dft_value) > 0.0f) relative_error = max_abs_diff / fabsf(dft_value);

   // Fail validation if DFT/FFT differ by more than 0.1%
   const float max_relative_error = 0.001f;

   if (relative_error > max_relative_error)
   {
      ESP_LOGE(
         LOG_TAG,
         "DFT/FFT validation FAILED: relative error %.6f%% exceeds %.3f%%",
         relative_error * 100.0f,
         max_relative_error * 100.0f
      );

      return ESP_FAIL;
   }

   ESP_LOGI(
      LOG_TAG,
      "DFT/FFT comparison: bin=%u DFT=%.3f FFT=%.3f abs_diff=%.3f relative_error=%.6f%%",
      (unsigned)max_diff_bin,
      dft_value,
      fft_value,
      max_abs_diff,
      relative_error * 100.0f
   );

   return ESP_OK;
}

static esp_err_t validate_expected_peaks(
   const char *transform_name,
   const float *mags,
   size_t bins,
   const size_t *expected_bins,
   size_t expected_bin_count
)
{
   if (transform_name == NULL ||
      mags == NULL || 
      expected_bins == NULL ||
      bins == 0 ||
      expected_bin_count == 0) return ESP_ERR_INVALID_ARG;

   for (size_t i = 0; i < expected_bin_count; i++)
   {
      if (expected_bins[i] >= bins)
      {
         ESP_LOGE(
            LOG_TAG,
            "%s expected bin %u is outside available bins=%u",
            transform_name,
            (unsigned)expected_bins[i],
            (unsigned)bins
         );

         return ESP_ERR_INVALID_ARG;
      }

      size_t expected_bin = expected_bins[i];
      float expected_mag = mags[expected_bin];

      float left_mag = (expected_bin > 0) ?
         mags[expected_bin - 1] : 0.0f;

      float right_mag = (expected_bin + 1 < bins) ?
         mags[expected_bin + 1] : 0.0f;

      if (expected_mag <= left_mag || expected_mag <= right_mag)
      {
         ESP_LOGE(
            LOG_TAG,
            "%s expected peak FAILED at bin=%u: left=%.3f peak=%.3f right=%.3f",
            transform_name,
            (unsigned)expected_bin,
            left_mag,
            expected_mag,
            right_mag
         );

         return ESP_FAIL;
      }

      ESP_LOGI(
         LOG_TAG,
         "%s expected peak PASSED at bin=%u: left=%.3f peak=%.3f right=%.3f",
         transform_name,
         (unsigned)expected_bin,
         left_mag,
         expected_mag,
         right_mag
      );
   }
   return ESP_OK;
}

static esp_err_t led_cube_mapper_self_test(void)
{
   const cube_mapper_test_case_t test_cases[] =
   {
   {0, 0, 0, 0, 0},    // LED 1
   {7, 0, 0, 0, 7},    // LED 8
   {7, 1, 0, 0, 8},    // LED 9
   {0, 1, 0, 0, 15},   // LED 16
   {0, 2, 0, 0, 16},   // LED 17
   {7, 2, 0, 0, 23},   // LED 24
   {7, 3, 0, 0, 24},   // LED 25
   {0, 3, 0, 0, 31},   // LED 32
   {0, 7, 0, 0, 63},   // LED 64
   {3, 4, 6, 6, 35}    // Arbitrary XYZ/layer check
};

   const size_t test_case_count = sizeof(test_cases) / sizeof(test_cases[0]);

   for (size_t i = 0; i < test_case_count; i++)
   {
      size_t layer = 0;
      size_t pixel_index = 0;

      esp_err_t status = led_cube_map_voxel(
         test_cases[i].x,
         test_cases[i].y,
         test_cases[i].z,
         &layer,
         &pixel_index
      );
      if (status != ESP_OK) return status;

      if (layer != test_cases[i].expected_layer ||
         pixel_index != test_cases[i].expected_pixel_index)
      {
         ESP_LOGE(
            LOG_TAG,
            "cube mapper FAILED: (%u,%u,%u) -> layer=%u index=%u, expected layer=%u index=%u",
            (unsigned)test_cases[i].x,
            (unsigned)test_cases[i].y,
            (unsigned)test_cases[i].z,
            (unsigned)layer,
            (unsigned)pixel_index,
            (unsigned)test_cases[i].expected_layer,
            (unsigned)test_cases[i].expected_pixel_index
         );

         return ESP_FAIL;
      }
   }

   ESP_LOGI(
      LOG_TAG,
      "cube mapper self-test PASSED: %u cases",
      (unsigned)test_case_count
   );

   return ESP_OK;
}

static esp_err_t audio_buffer_self_test(void)
{
   audio_buffer_init();
   
   const size_t expected_bins[] = {4, 20};
   const float test_amplitudes[] = {1200.0f, 700.0f};
   const size_t expected_bin_count = sizeof(expected_bins) / sizeof(expected_bins[0]);

   const int16_t *frame = NULL;
   // Push audio samples to complete a frame
   esp_err_t status = generate_test_audio_frame(
      expected_bins,
      test_amplitudes,
      expected_bin_count,
      &frame
   );
   if (status != ESP_OK) return status;

   size_t dft_bins = 0;
   const float *dft_mags = NULL;

   // -------------------------------
   // ---- DFT/FFT validation ----
   // -------------------------------

   status = run_dft_test(
      frame,
      &dft_mags,
      &dft_bins
   );
   if (status != ESP_OK) return status;
   log_transform_test_result("DFT", dft_mags, dft_bins);

   status = validate_expected_peaks(
      "DFT",
      dft_mags,
      dft_bins,
      expected_bins,
      expected_bin_count
   );
   if (status != ESP_OK) return status;

   size_t fft_bins = 0;
   const float *fft_mags = NULL;

   status = run_fft_test(
      frame,
      &fft_mags,
      &fft_bins
   );
   if (status != ESP_OK) return status;
   log_transform_test_result("FFT", fft_mags, fft_bins);

   status = validate_expected_peaks(
      "FFT",
      fft_mags,
      fft_bins,
      expected_bins,
      expected_bin_count
   );
   if (status != ESP_OK) return status;

   status = compare_dft_fft(
      dft_mags,
      dft_bins,
      fft_mags,
      fft_bins
   );

   if (status != ESP_OK) return status;

   status = test_spectrum_mapping(
      dft_mags,
      dft_bins
   );
   if (status != ESP_OK) return status;
   
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
   esp_err_t status = led_cube_mapper_self_test();
   if (status!= ESP_OK) return status;

   // status = audio_buffer_self_test();
   // if (status != ESP_OK) return status;

   return ESP_OK;
}
