#include "fft_engine.h"
#include "common/math_constants.h"

// ------ Module State
static float s_magnitudes[FFT_NUM_BINS];
static float s_window[FFT_FRAME_SIZE];
static size_t s_num_bins = FFT_NUM_BINS;
static bool s_initialized = false;

// Initialize empty transform frame
esp_err_t fft_engine_init(void)
{
   for (size_t i = 0; i < s_num_bins; i++) s_magnitudes[i] = 0.0f;
   for (size_t n = 0; n < FFT_FRAME_SIZE; n++)
   {
      // Hann window:
      // w[n] = 0.5 - 0.5*cos(2*pi*n/(N-1))
      s_window[n] =
         0.5f - 0.5f * cosf((MATH_TWO_PI * (float)n) / (float)(FFT_FRAME_SIZE - 1));
   }
   s_initialized = true;
   return ESP_OK;
}

esp_err_t fft_engine_process_frame(const int16_t *frame)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE; 
   if (frame == NULL) return ESP_ERR_INVALID_ARG;

   // Remove per-frame DC offset so low-frequency bins do not dominate.
   float frame_mean = 0.0f;
   for (size_t n = 0; n < FFT_FRAME_SIZE; n++)
   {
      frame_mean += (float)frame[n];
   }
   // Frame mean (DC estimate):
   // mean = (1/N) * sum(frame[n])
   frame_mean /= (float)FFT_FRAME_SIZE;

   for (size_t i = 0; i < s_num_bins; i++)
   {
      float real_sum = 0.0f; // Real accumulator for freq bin [i]
      float imag_sum = 0.0f; // Imaginary accumulator for freq cin [i]
      float delta = (MATH_TWO_PI * (float)i) / (float)FFT_FRAME_SIZE;
      float cos_delta = cosf(delta);
      float sin_delta = sinf(delta);
      float cos_n = 1.0f;
      float sin_n = 0.0f;

      for (size_t n = 0; n < FFT_FRAME_SIZE; n++)
      {
         float x = (float)frame[n] - frame_mean; // Center signal around zero
         float windowed_sample = x * s_window[n];

         // DFT projection:
         // Re[k] += xw[n]*cos(2*pi*k*n/N)
         // Im[k] -= xw[n]*sin(2*pi*k*n/N)
         // Project sample onto cosine/sine basis for current n
         real_sum += windowed_sample * cos_n;
         imag_sum -= windowed_sample * sin_n;

         // Advance oscillator by one step:
         // e^{-j(n+1)delta} from e^{-jndelta}
         float next_cos_n = (cos_n * cos_delta) - (sin_n * sin_delta);
         float next_sin_n = (sin_n * cos_delta) + (cos_n * sin_delta);
         cos_n = next_cos_n;
         sin_n = next_sin_n;
      }  
         // Magnitude:
         // |X[k]| = sqrt(Re[k]^2 + Im[k]^2)
         s_magnitudes[i] = sqrtf((real_sum * real_sum) + (imag_sum * imag_sum));
   } 

   return ESP_OK;
}

esp_err_t fft_engine_get_magnitudes(const float **mags, size_t *num_bins)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE; 
   if (mags == NULL || num_bins == NULL) return ESP_ERR_INVALID_ARG;

   *mags = s_magnitudes; // object of magnitude readings
   *num_bins = s_num_bins;
   return ESP_OK;
}
