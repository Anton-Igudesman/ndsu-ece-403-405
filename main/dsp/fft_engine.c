#include "common/math_constants.h"
#include "fft_engine.h"

#include <math.h>
#include <stdint.h>

// ---------------- Module State
static float s_magnitudes[FFT_NUM_BINS];
static float s_window[FFT_FRAME_SIZE];

static float s_real[FFT_FRAME_SIZE];
static float s_imag[FFT_FRAME_SIZE];

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

// Reverse bits to make adjacent pairs for butterfly operations
static void fft_bit_reverse(void)
{
   size_t j = 0;

   for (size_t i = 1; i < FFT_FRAME_SIZE; i++)
   {
      size_t bit = FFT_FRAME_SIZE >> 1;

      // Bit reversing logic
      while (j & bit)
      {
         j ^= bit;
         bit >>= 1;
      }

      j ^= bit;

      // Only swap when i < j to make sure no double swaps happen
      if (i < j)
      {
         float temp_real = s_real[i];
         s_real[i] = s_real[j];
         s_real[j] = temp_real;

         float temp_imag = s_imag[i];
         s_imag[i] = s_imag[j];
         s_imag[j] = temp_imag;
      }
   }
}

static float fft_calculate_frame_mean(const int16_t *frame)
{
   // Remove per-frame DC offset so low-frequency bins do not dominate.
   float frame_mean = 0.0f;
   for (size_t n = 0; n < FFT_FRAME_SIZE; n++) frame_mean += (float)frame[n];
   
   // Frame mean (DC estimate):
   // mean = (1/N) * sum(frame[n])
   frame_mean /= (float)FFT_FRAME_SIZE;

   return frame_mean;
}

static void fft_prepare_input(const int16_t *frame, float frame_mean)
{
   for (size_t n = 0; n < FFT_FRAME_SIZE; n++)
   {
      float x = (float)frame[n] - frame_mean;
      s_real[n] = x * s_window[n];
      s_imag[n] = 0.0f;
   }
}

static void fft_execute(void)
{
   // Double length of FFT group each iteration
   // 10 stages of radix-2 FFT
   for (size_t len = 2; len <= FFT_FRAME_SIZE; len <<= 1)
   {
      // Twiddle factor = W = cos(angle) + jsin(angle)
      float angle = -MATH_TWO_PI / (float)len;
      float wlen_real = cosf(angle);
      float wlen_imag = sinf(angle);
      
      // Process each group of samples in current stage
      for (size_t i = 0; i < FFT_FRAME_SIZE; i += len)
      {
         float w_real = 1.0f;
         float w_imag = 0.0f;

         // Perform butterfly
         for (size_t j = 0; j < len / 2; j++)
         {
            size_t even_index = i + j; 
            size_t odd_index = i + j + (len / 2);

            // (ac - bd) + j(ad + bc)
            float t_real = (w_real * s_real[odd_index]) - (w_imag * s_imag[odd_index]);
            float t_imag = (w_real * s_imag[odd_index]) + (w_imag * s_real[odd_index]);

            float u_real = s_real[even_index];
            float u_imag = s_imag[even_index];

            s_real[even_index] = u_real + t_real;
            s_imag[even_index] = u_imag + t_imag;

            s_real[odd_index] = u_real - t_real;
            s_imag[odd_index] = u_imag - t_imag;

            // Advance twiddle factor for next butterfly
            float next_w_real = (w_real * wlen_real) - (w_imag * wlen_imag);
            float next_w_imag = (w_real * wlen_imag) + (w_imag * wlen_real);

            w_real = next_w_real;
            w_imag = next_w_imag;
         }
      }
   }
}

static void fft_calculate_magnitudes(void)
{
   for (size_t k = 0; k < FFT_NUM_BINS; k++)
   {
      float real = s_real[k];
      float imag = s_imag[k];

      s_magnitudes[k] = sqrtf((real * real) + (imag * imag));
   }
}
esp_err_t fft_engine_process_frame(const int16_t *frame)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE; 
   if (frame == NULL) return ESP_ERR_INVALID_ARG;

   float frame_mean = fft_calculate_frame_mean(frame);

   // Load 1024 audio samples into complex FFT arrays
   fft_prepare_input(frame, frame_mean);

   fft_bit_reverse();
   fft_execute();
   fft_calculate_magnitudes();
   
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