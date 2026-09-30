#include "dft_trace.h"

#include <math.h>

#include "common/math_constants.h"
#include "dft_engine.h"
#include "esp_log.h"

static const char *LOG_TAG = "dft_trace";

static void dft_trace_bins(
   const int16_t *frame,
   float frame_mean,
   float *trace_re_k10,
   float *trace_im_k10,
   float *trace_re_k40,
   float *trace_im_k40,
   float *trace_re_k100,
   float *trace_im_k100
)
{
   const size_t trace_bins[] = {10U, 40U, 100U};

   for (size_t i = 0; i < 3U; i++)
   {
      const size_t k = trace_bins[i];
      float real_sum;
      float imag_sum;

      dft_calculate_bin(
         frame,
         frame_mean,
         k,
         &real_sum,
         &imag_sum
      );

      if (k == 10U)
      {
         *trace_re_k10 = real_sum;
         *trace_im_k10 = imag_sum;
      }
      else if (k == 40U)
      {
         *trace_re_k40 = real_sum;
         *trace_im_k40 = imag_sum;
      }
      else if (k == 100U)
      {
         *trace_re_k100 = real_sum;
         *trace_im_k100 = imag_sum;
      }
   }
}

static void dft_trace_first_terms_k10(
   const int16_t *frame,
   float frame_mean,
   float *re_n0,
   float *im_n0,
   float *re_n1,
   float *im_n1,
   float *re_n2,
   float *im_n2
)
{
   const size_t k = 10U;

   float delta =
      (MATH_TWO_PI * (float)k) /
      (float)DFT_FRAME_SIZE;

   float cos_delta = cosf(delta);
   float sin_delta = sinf(delta);

   float cos_n = 1.0f;
   float sin_n = 0.0f;

   for (size_t n = 0; n < 3U; n++)
   {
      float x = (float)frame[n] - frame_mean;

      float window =
         0.5f -
         0.5f * cosf(
            (MATH_TWO_PI * (float)n) /
            (float)(DFT_FRAME_SIZE - 1)
         );

      float windowed_sample = x * window;

      float re_term = windowed_sample * cos_n;
      float im_term = -windowed_sample * sin_n;

      if (n == 0U)
      {
         *re_n0 = re_term;
         *im_n0 = im_term;
      }
      else if (n == 1U)
      {
         *re_n1 = re_term;
         *im_n1 = im_term;
      }
      else
      {
         *re_n2 = re_term;
         *im_n2 = im_term;
      }

      float next_cos_n =
         (cos_n * cos_delta) -
         (sin_n * sin_delta);

      float next_sin_n =
         (sin_n * cos_delta) +
         (cos_n * sin_delta);

      cos_n = next_cos_n;
      sin_n = next_sin_n;
   }
}

void dft_trace_frame(const int16_t *frame, float frame_mean)
{
   float trace_re_k10 = 0.0f;
   float trace_im_k10 = 0.0f;
   float trace_re_k40 = 0.0f;
   float trace_im_k40 = 0.0f;
   float trace_re_k100 = 0.0f;
   float trace_im_k100 = 0.0f;

   float trace_re_term_k10_n0 = 0.0f;
   float trace_im_term_k10_n0 = 0.0f;
   float trace_re_term_k10_n1 = 0.0f;
   float trace_im_term_k10_n1 = 0.0f;
   float trace_re_term_k10_n2 = 0.0f;
   float trace_im_term_k10_n2 = 0.0f;

   dft_trace_bins(
      frame,
      frame_mean,
      &trace_re_k10,
      &trace_im_k10,
      &trace_re_k40,
      &trace_im_k40,
      &trace_re_k100,
      &trace_im_k100
   );

   dft_trace_first_terms_k10(
      frame,
      frame_mean,
      &trace_re_term_k10_n0,
      &trace_im_term_k10_n0,
      &trace_re_term_k10_n1,
      &trace_im_term_k10_n1,
      &trace_re_term_k10_n2,
      &trace_im_term_k10_n2
   );

   float x0 = (float)frame[0] - frame_mean;
   float x1 = (float)frame[1] - frame_mean;
   float x2 = (float)frame[2] - frame_mean;

   float window0 =
      0.5f -
      0.5f * cosf(
         0.0f
      );

   float window1 =
      0.5f -
      0.5f * cosf(
         (MATH_TWO_PI * 1.0f) /
         (float)(DFT_FRAME_SIZE - 1)
      );

   float window2 =
      0.5f -
      0.5f * cosf(
         (MATH_TWO_PI * 2.0f) /
         (float)(DFT_FRAME_SIZE - 1)
      );

   float xw0 = x0 * window0;
   float xw1 = x1 * window1;
   float xw2 = x2 * window2;

   const float hz_per_bin =
      48000.0f / (float)DFT_FRAME_SIZE;

   const float k10_hz = 10.0f * hz_per_bin;
   const float k40_hz = 40.0f * hz_per_bin;
   const float k100_hz = 100.0f * hz_per_bin;

   float k10_mag =
      sqrtf(
         (trace_re_k10 * trace_re_k10) +
         (trace_im_k10 * trace_im_k10)
      );

   float k40_mag =
      sqrtf(
         (trace_re_k40 * trace_re_k40) +
         (trace_im_k40 * trace_im_k40)
      );

   float k100_mag =
      sqrtf(
         (trace_re_k100 * trace_re_k100) +
         (trace_im_k100 * trace_im_k100)
      );

   ESP_LOGI(
      LOG_TAG,
      "DFT step 1 (DC removal + window): frame mean/DC estimate = %.2f; first centered samples x[n] = [%.2f, %.2f, %.2f]; first windowed samples x_w[n] = [%.2f, %.2f, %.2f]",
      frame_mean,
      x0,
      x1,
      x2,
      xw0,
      xw1,
      xw2
   );

   ESP_LOGI(
      LOG_TAG,
      "DFT step 2 (term accumulation example for bin k=10): first 3 term contributions are n0(Re=%.2f, Im=%.2f), n1(Re=%.2f, Im=%.2f), n2(Re=%.2f, Im=%.2f)",
      trace_re_term_k10_n0,
      trace_im_term_k10_n0,
      trace_re_term_k10_n1,
      trace_im_term_k10_n1,
      trace_re_term_k10_n2,
      trace_im_term_k10_n2
   );

   ESP_LOGI(
      LOG_TAG,
      "DFT step 3 (bin k=10): frequency = %.2f Hz; Re = %.2f; Im = %.2f; magnitude = %.2f",
      k10_hz,
      trace_re_k10,
      trace_im_k10,
      k10_mag
   );

   ESP_LOGI(
      LOG_TAG,
      "DFT step 3 (bin k=40): frequency = %.2f Hz; Re = %.2f; Im = %.2f; magnitude = %.2f",
      k40_hz,
      trace_re_k40,
      trace_im_k40,
      k40_mag
   );

   ESP_LOGI(
      LOG_TAG,
      "DFT step 3 (bin k=100): frequency = %.2f Hz; Re = %.2f; Im = %.2f; magnitude = %.2f",
      k100_hz,
      trace_re_k100,
      trace_im_k100,
      k100_mag
   );
}