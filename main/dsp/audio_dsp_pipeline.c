#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "audio_buffer.h"
#include "fft_engine.h"
#include "audio_dsp_pipeline.h"

#define AUDIO_SAMPLE_RATE_HZ 48000.0f

// Reduce persistent low-band domincance that can flatten EQ shape
// Possibly at the loss of bass detail below 10 * 46.875 Hz
#define FFT_SKIP_LOW_BINS 10
#define DSP_PIPELINE_TRACE_LOGS 1
#define DSP_PIPELINE_TRACE_DIVISOR 32U

static const char *LOG_TAG = "audio_dsp";

// Enable sharing frequency information across modules
static float s_last_band_peak_hz[SPECTRUM_NUM_BANDS] = {0};
static float s_last_band_peak_mag[SPECTRUM_NUM_BANDS] = {0};
static bool s_last_band_peaks_valid = false;
static uint32_t s_trace_frame_counter = 0;

// API serves consumers that need latest frequency state
esp_err_t audio_dsp_get_last_band_peaks(
   float *band_peak_hz_out,
   float *band_peak_mag_out,
   size_t num_bands)
{
   if (band_peak_hz_out == NULL || 
      band_peak_mag_out == NULL ||
      num_bands == 0 ||
      num_bands > SPECTRUM_NUM_BANDS) return ESP_ERR_INVALID_ARG;

   // No cache available until at least one successful per-band peak compute
   if (!s_last_band_peaks_valid) return ESP_ERR_INVALID_STATE;

   // Copy cached telemetry for external consumers
   for (size_t i = 0; i < num_bands; i++)
   {
      band_peak_hz_out[i] = s_last_band_peak_hz[i];
      band_peak_mag_out[i] = s_last_band_peak_mag[i];
   }

   return ESP_OK;
}

esp_err_t audio_dsp_get_peak_frequency(
   float *peak_hz_out,
   size_t *peak_bin_out,
   float *peak_mag_out)
{
   if (peak_hz_out == NULL || 
      peak_bin_out == NULL || 
      peak_mag_out == NULL) return ESP_ERR_INVALID_ARG; // Validate output pointers
   
   const float *mags = NULL;
   size_t bins = 0;

   esp_err_t status = fft_engine_get_magnitudes(&mags, &bins);
   if (status != ESP_OK) return status; // Propagate FFT state/read error
   
   if (mags == NULL || bins == 0) return ESP_ERR_INVALID_STATE; // No usable spectrum data
   

   size_t start_bin = FFT_SKIP_LOW_BINS;
   if (start_bin >= bins) start_bin = 0; // Fallback when skip exceeds available bins
   
   size_t peak_bin = start_bin;
   float peak_mag = mags[start_bin];

   for (size_t k = start_bin + 1; k < bins; k++)
   {
      if (mags[k] > peak_mag)
      {
         peak_mag = mags[k];
         peak_bin = k; // Track strongest bin
      }
   }

   const float hz_per_bin = AUDIO_SAMPLE_RATE_HZ / (float)FFT_FRAME_SIZE; // Hz spacing between bins
   const float peak_hz = (float)peak_bin * hz_per_bin; // Peak bin index -> Hz

   *peak_hz_out = peak_hz;
   *peak_bin_out = peak_bin;
   *peak_mag_out = peak_mag;

   return ESP_OK;
}

esp_err_t audio_dsp_get_band_peak_frequency(
   float *band_peak_hz_out,
   float *band_peak_mag_out,
   size_t num_bands)
{
   if (band_peak_hz_out == NULL || 
      band_peak_mag_out == NULL || 
      num_bands == 0 ||
      num_bands > SPECTRUM_NUM_BANDS) return ESP_ERR_INVALID_ARG;

   const float *mags = NULL;
   size_t bins = 0;

   esp_err_t status = fft_engine_get_magnitudes(&mags, &bins);
   if (status != ESP_OK) return status;
   
   // No usable spectrum data
   if (mags == NULL || bins == 0) return ESP_ERR_INVALID_STATE;
   
   size_t start_offset = FFT_SKIP_LOW_BINS;

   // Fallback when skip exceeds available bins 
   if (start_offset >= bins) start_offset = 0;

   const float *mapped_bins = mags + start_offset;
   size_t mapped_bins_count = bins - start_offset;

   if (mapped_bins_count == 0) return ESP_ERR_INVALID_STATE;

   for (size_t band = 0; band < num_bands; band++)
   {
      // Log-spaced edge match 
      const float min_bin_index_f = 1.0f;
      const float max_bin_index_f = (float)mapped_bins_count;
      const float log_min = logf(min_bin_index_f);
      const float log_max = logf(max_bin_index_f);

      const float t0 = (float)band / (float)num_bands;
      const float t1 = (float)(band + 1U) / (float)num_bands;

      size_t start = (size_t)floorf(expf(log_min + t0 * (log_max - log_min)));
      size_t end = (size_t)floorf(expf(log_min + t1 * (log_max - log_min)));

      // Catch bin state edge cases
      if (start >= mapped_bins_count) start = mapped_bins_count - 1U;
      if (end > mapped_bins_count) end = mapped_bins_count;
      if (end <= start) end = start + 1U;
      if (end > mapped_bins_count) end = mapped_bins_count;

      size_t local_peak_idx = start;
      float local_peak_mag = mapped_bins[start];

      for (size_t i = start + 1U; i < end; i++)
      {
         if (mapped_bins[i] > local_peak_mag)
         {
            local_peak_mag = mapped_bins[i];
            local_peak_idx = i;
         }
      }
      size_t global_bin = local_peak_idx + start_offset;
      const float hz_per_bin = AUDIO_SAMPLE_RATE_HZ / (float)FFT_FRAME_SIZE;

      band_peak_hz_out[band] = (float)global_bin * hz_per_bin;
      band_peak_mag_out[band] = local_peak_mag;

      // Save to static module scope variables
      s_last_band_peak_hz[band] = band_peak_hz_out[band];
      s_last_band_peak_mag[band] = band_peak_mag_out[band];
   }

   s_last_band_peaks_valid = true;
   return ESP_OK;
}

static esp_err_t compute_transform_magnitudes_from_frame(
   const int16_t *frame,
   const float **mags_out,
   size_t *bins_out)
{
   // Validate input/output pointers before DSP state
   if (frame == NULL || mags_out == NULL || bins_out == NULL) return ESP_ERR_INVALID_ARG;

   // FFT engine stores computed magnitudes in internal module buffer
   esp_err_t status = fft_engine_process_frame(frame);
   if (status != ESP_OK) return status; 

   // Read back pointer and bin count from FFT engine internal state
   const float *mags = NULL;
   size_t bins = 0;
   status = fft_engine_get_magnitudes(&mags, &bins);
   if (status != ESP_OK) return status;

   // Unexpected empty FFT output
   if (mags == NULL || bins == 0) return ESP_ERR_INVALID_STATE;

   // Return transform outputs to caller 
   *mags_out = mags;
   *bins_out = bins;
   return ESP_OK;
}

// Map FFT magnitudes into fixed number of spectrum bands
static esp_err_t map_fft_bins_to_bands(
   const float *mags,
   size_t bins,
   float *bands_out)
{
   // Validate required inputs and output buffer
   if (mags == NULL || bands_out == NULL || bins == 0) return ESP_ERR_INVALID_ARG;

   // Skip DC and near-DC bins to reduce low-freq bias
   size_t mapped_bins = 0;
   if (bins > FFT_SKIP_LOW_BINS) mapped_bins = bins - FFT_SKIP_LOW_BINS;

   if (mapped_bins == 0) return ESP_ERR_INVALID_STATE;

   // Map selected FFT region into SPECTRUM_NUM_BANDS averaged bands
   return spectrum_map_bins_to_bands(
      mags + FFT_SKIP_LOW_BINS,
      mapped_bins,
      bands_out,
      SPECTRUM_NUM_BANDS
   );
}

// Apply soft gate and fixed floor suppression to raw bands in-place
static void apply_soft_gate_and_floor(
   float *bands,
   size_t num_bands,
   float *gate_gain_out,
   float *max_raw_out)
{
   if (bands == NULL || num_bands == 0) return;

   // Clamp num_bands to SPECTRUM_NUM_BANDS
   if (num_bands > SPECTRUM_NUM_BANDS) num_bands = SPECTRUM_NUM_BANDS;

   // Find peak raw band value for this frame
   float max_raw = 0.0f;
   for (size_t i = 0; i < num_bands; i++)
   {
      if (bands[i] > max_raw) max_raw = bands[i];
   } 

   /*
      Soft gate
      gain = clamp((max_raw - gate_lo) / (gate_hi - gate_lo), 0..1)
   */
   const float gate_lo = 120.0f;
   const float gate_hi = 300.0f;
   float gate_gain = 0.0f;

   if (max_raw <= gate_lo) gate_gain = 0.0f;
   else if (max_raw >= gate_hi) gate_gain = 1.0f;
   else gate_gain = (max_raw - gate_lo) / (gate_hi - gate_lo);

   for (size_t i = 0; i < num_bands; i++) bands[i] *= gate_gain;

   /*
      Subtract floor to suppress broadband noise haze
      band = max(0, band - noise_floor)
   */
   const float noise_floor = 110.0f;
   for (size_t i = 0; i < num_bands; i++)
   {
      bands[i] = bands[i] - noise_floor;
      if (bands[i] < 0.0f) bands[i] = 0.0f;
   }

   if (gate_gain_out != NULL) *gate_gain_out = gate_gain;
   if (max_raw_out != NULL) *max_raw_out = max_raw;
   
}

// Smooth raw conditioned bands in-place with per-band EMA
static void smooth_raw_bands(float *bands, size_t num_bands)
{
   if (bands == NULL || num_bands == 0) return;

   if (num_bands > SPECTRUM_NUM_BANDS) num_bands = SPECTRUM_NUM_BANDS;

   // Persistent per-band state across frames
   static bool s_raw_smooth_initialized = false;
   static float s_raw_bands_smooth[SPECTRUM_NUM_BANDS] = {0};
   const float raw_smoothing_alpha = 0.12f;

   /*
      EMA:
      y[n] = y[n-1] + alpha * (x[n] - y[n-1])
   */
   for (size_t i = 0; i < num_bands; i++)
   {
      float current = bands[i];
      float prev = s_raw_smooth_initialized ? s_raw_bands_smooth[i] : current;
      float smooth = prev + raw_smoothing_alpha * (current - prev);

      s_raw_bands_smooth[i] = smooth;
      bands[i] = smooth;
   }
   s_raw_smooth_initialized = true;
}

// Normalize conditioned bands into [0,1] range using running-max envelope
static void normalize_bands_running_max(
   const float *bands_in,
   size_t num_bands,
   float *bands_out,
   float *running_max_out,
   float *frame_max_out)
{
   if (bands_in == NULL || bands_out == NULL || num_bands == 0) return;

   if (num_bands > SPECTRUM_NUM_BANDS) num_bands = SPECTRUM_NUM_BANDS;

   // Persistent envelope state across frames
   static float s_running_max = 1.0f;

   // Frame peak from conditioned/smoothed bands
   float frame_max = 0.0f;
   for (size_t i = 0; i < num_bands; i++)
   {
      if (bands_in[i] > frame_max) frame_max = bands_in[i];
   }

   // Fast rise, slower decay envelope tracker
   // Helps prevent persistent top-band pegging after transients
   const float running_max_decay = 0.985f;

   // Allows fast rise with slow decay as quieter sounds come in
   if (frame_max > s_running_max) s_running_max = frame_max;
   else
   {
      s_running_max *= running_max_decay;

      // Prevent normalization denom from collapsing too low during quiet
      if (s_running_max < 50.0f) s_running_max = 50.0f;
   }

   // Clamp normalized output to [0,1]
   for (size_t i = 0; i < num_bands; i++)
   {
      float norm = bands_in[i] / s_running_max;
      if (norm < 0.0f) norm = 0.0f;
      if (norm > 1.0f) norm = 1.0f;
      bands_out[i] = norm;
   }

   if (running_max_out != NULL) *running_max_out = s_running_max;
   if (frame_max_out != NULL) *frame_max_out = frame_max;
}

static void trace_pipeline_math_once(
   size_t bins,
   const float *bands_mapped,
   const float *bands_conditioned,
   const float *bands_norm,
   float max_raw,
   float gate_gain,
   float frame_max,
   float running_max)
{
   if (bands_mapped == NULL || bands_conditioned == NULL || bands_norm == NULL) return;

   s_trace_frame_counter++;
   if ((s_trace_frame_counter % DSP_PIPELINE_TRACE_DIVISOR) != 0U) return;

   ESP_LOGI(
      LOG_TAG,
      "DSP conditioning step: FFT bins=%u, skipped low bins=%u, mapped bins=%u, strongest raw band=%.2f, soft-gate gain=%.3f, conditioned frame max=%.2f, running normalization max=%.2f",
      (unsigned)bins,
      (unsigned)FFT_SKIP_LOW_BINS,
      (unsigned)((bins > FFT_SKIP_LOW_BINS) ? (bins - FFT_SKIP_LOW_BINS) : 0U),
      max_raw,
      gate_gain,
      frame_max,
      running_max
   );

   ESP_LOGI(
      LOG_TAG,
      "DSP mapping result (pre-conditioning), 8 band means: [%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f]",
      bands_mapped[0], bands_mapped[1], bands_mapped[2], bands_mapped[3],
      bands_mapped[4], bands_mapped[5], bands_mapped[6], bands_mapped[7]
   );

   ESP_LOGI(
      LOG_TAG,
      "DSP conditioning result (after gate/floor/smoothing), 8 bands: [%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f]",
      bands_conditioned[0], bands_conditioned[1], bands_conditioned[2], bands_conditioned[3],
      bands_conditioned[4], bands_conditioned[5], bands_conditioned[6], bands_conditioned[7]
   );

   ESP_LOGI(
      LOG_TAG,
      "DSP final normalized output for LED/BLE (0..1), 8 bands: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
      bands_norm[0], bands_norm[1], bands_norm[2], bands_norm[3],
      bands_norm[4], bands_norm[5], bands_norm[6], bands_norm[7]
   );
}

// run FFT -> map -> condition -> smooth -> normalize
static esp_err_t map_condition_normalize_bands(
   const float *mags,
   size_t bins,
   float *bands_norm_out)
{
   if (mags == NULL || bands_norm_out == NULL || bins == 0) return ESP_ERR_INVALID_ARG;

   float bands[SPECTRUM_NUM_BANDS] = {0};
   float bands_mapped[SPECTRUM_NUM_BANDS] = {0};
   float bands_conditioned[SPECTRUM_NUM_BANDS] = {0};
   float gate_gain = 0.0f;
   float max_raw = 0.0f;
   float running_max = 0.0f;
   float frame_max = 0.0f;

   esp_err_t status = map_fft_bins_to_bands(mags, bins, bands);
   if (status != ESP_OK) return status;

   for (size_t band_index = 0; band_index < SPECTRUM_NUM_BANDS; band_index++)
   {
      bands_mapped[band_index] = bands[band_index];
   }

   apply_soft_gate_and_floor(bands, SPECTRUM_NUM_BANDS, &gate_gain, &max_raw);
   smooth_raw_bands(bands, SPECTRUM_NUM_BANDS);

   for (size_t band_index = 0; band_index < SPECTRUM_NUM_BANDS; band_index++)
   {
      bands_conditioned[band_index] = bands[band_index];
   }

   normalize_bands_running_max(
      bands,
      SPECTRUM_NUM_BANDS,
      bands_norm_out,
      &running_max,
      &frame_max
   );

#if DSP_PIPELINE_TRACE_LOGS
   trace_pipeline_math_once(
      bins,
      bands_mapped,
      bands_conditioned,
      bands_norm_out,
      max_raw,
      gate_gain,
      frame_max,
      running_max
   );
#endif

   return ESP_OK;
}

/*
   If a full frame is ready, run FFT -> map -> condition -> smooth -> normalize
   Returns ESP_OK when no error (whether or not frame was ready)
*/
esp_err_t audio_dsp_process_ready_frame(float *bands_norm_out, bool *frame_processed)
{
   if (bands_norm_out == NULL || frame_processed == NULL) return ESP_ERR_INVALID_ARG;

   *frame_processed = false;

   const int16_t *frame = NULL;
   esp_err_t status = audio_buffer_try_get_frame(&frame);
   if(status != ESP_OK) return status;

   if (frame == NULL) return ESP_OK;

   const float *mags = NULL;
   size_t bins = 0;
   status = compute_transform_magnitudes_from_frame(frame, &mags, &bins);
   if (status != ESP_OK) return status;

   status = map_condition_normalize_bands(mags, bins, bands_norm_out);
   if (status != ESP_OK) return status;

   *frame_processed = true;
   return ESP_OK;
}
