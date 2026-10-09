#include "common/math_constants.h"
#include "dft_engine.h"
#include "dft_trace.h"
#include <math.h>
#include <stdint.h>

#define DFT_ENGINE_TRACE_LOGS 1
#define DFT_ENGINE_TRACE_DIVISOR 32U

// ------ Module State
static float s_magnitudes[DFT_NUM_BINS];
static float s_window[DFT_FRAME_SIZE];
static size_t s_num_bins = DFT_NUM_BINS;
static bool s_initialized = false;
static uint32_t s_dft_trace_counter = 0;

void dft_calculate_bin(
   const int16_t *frame,
   float frame_mean,
   size_t k,
   float *real_sum,
   float *imag_sum
)
{
   *real_sum = 0.0f;
   *imag_sum = 0.0f;

   float delta = (MATH_TWO_PI * (float)k) / (float)DFT_FRAME_SIZE;

   float cos_delta = cosf(delta);
   float sin_delta = sinf(delta);
   float cos_n = 1.0f;
   float sin_n = 0.0f;

   for (size_t i = 0; i < DFT_FRAME_SIZE; i++)
   {
      float x = (float)frame[i] - frame_mean; // Center signal around zero
      float windowed_sample = x * s_window[i];

      // DFT projection:
      // Re[k] += xw[n]*cos(2*pi*k*n/N)
      // Im[k] -= xw[n]*sin(2*pi*k*n/N)
      // Project sample onto cosine/sine basis for current n
      float re_term = windowed_sample * cos_n;
      float im_term = -windowed_sample * sin_n;
      *real_sum += re_term;
      *imag_sum += im_term;

      // Advance oscillator by one step:
      // e^{-j(n+1)delta} from e^{-jndelta}
      float next_cos_n = (cos_n * cos_delta) - (sin_n * sin_delta);
      float next_sin_n = (sin_n * cos_delta) + (cos_n * sin_delta);
      cos_n = next_cos_n;
      sin_n = next_sin_n;
   }
}

// Initialize empty transform frame
esp_err_t dft_engine_init(void)
{
   for (size_t i = 0; i < s_num_bins; i++) s_magnitudes[i] = 0.0f;
   for (size_t n = 0; n < DFT_FRAME_SIZE; n++)
   {
      // Hann window:
      // w[n] = 0.5 - 0.5*cos(2*pi*n/(N-1))
      s_window[n] =
         0.5f - 0.5f * cosf((MATH_TWO_PI * (float)n) / (float)(DFT_FRAME_SIZE - 1));
   }
   s_initialized = true;
   return ESP_OK;
}

esp_err_t dft_engine_process_frame(const int16_t *frame)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE; 
   if (frame == NULL) return ESP_ERR_INVALID_ARG;

   // Remove per-frame DC offset so low-frequency bins do not dominate.
   float frame_mean = 0.0f;
   for (size_t n = 0; n < DFT_FRAME_SIZE; n++) frame_mean += (float)frame[n];
   
   // Frame mean (DC estimate):
   // mean = (1/N) * sum(frame[n])
   frame_mean /= (float)DFT_FRAME_SIZE;

   for (size_t i = 0; i < s_num_bins; i++)
   {
      float real_sum = 0.0f; // Real accumulator for freq bin [i]
      float imag_sum = 0.0f; // Imaginary accumulator for freq cin [i]
      
      dft_calculate_bin(
         frame,
         frame_mean,
         i,
         &real_sum,
         &imag_sum
      );

      // Magnitude:
      // |X[k]| = sqrt(Re[k]^2 + Im[k]^2)
      s_magnitudes[i] = sqrtf((real_sum * real_sum) + (imag_sum * imag_sum));
   }

#if DFT_ENGINE_TRACE_LOGS
   s_dft_trace_counter++;
   if ((s_dft_trace_counter % DFT_ENGINE_TRACE_DIVISOR) == 0U) dft_trace_frame(frame, frame_mean);
#endif

   return ESP_OK;
}

esp_err_t dft_engine_get_magnitudes(const float **mags, size_t *num_bins)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE; 
   if (mags == NULL || num_bins == NULL) return ESP_ERR_INVALID_ARG;

   *mags = s_magnitudes; // object of magnitude readings
   *num_bins = s_num_bins;
   return ESP_OK;
}
