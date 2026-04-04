#include "led_matrix_renderer.h"

#define MATRIX_WIDTH 8U
#define MATRIX_HEIGHT 8U
#define EQ_PIXEL_G 32U

static led_strip_handle_t s_led_strip = NULL;

static uint32_t matrix_xy_to_index(uint8_t x, uint8_t y)
{
   // Uniform column mapping with bottom->top visual direction.
   return ((uint32_t)x * MATRIX_HEIGHT) + (MATRIX_HEIGHT - 1U - y);
}

static float clamp01(float value)
{
   if (value < 0.0f) return 0.0f;
   if (value > 1.0f) return 1.0f;
   return value;
}

static uint8_t band_to_height(float normalized_band_level)
{
   float clamped = clamp01(normalized_band_level);
   uint8_t height = (uint8_t)(clamped * (float)MATRIX_HEIGHT + 0.5f);
   if (height > MATRIX_HEIGHT) height = MATRIX_HEIGHT;
   return height;
}

static esp_err_t clear_matrix_frame(void)
{
   for (uint8_t y = 0; y < MATRIX_HEIGHT; y++)
   {
      for (uint8_t x = 0; x < MATRIX_WIDTH; x++)
      {
         uint32_t idx = matrix_xy_to_index(x, y);
         esp_err_t status = led_strip_set_pixel(s_led_strip, idx, 0, 0, 0);
         if (status != ESP_OK) return status;
      }
   }
   return ESP_OK;
}

static esp_err_t draw_eq_columns(const float *bands_norm)
{
   for (uint8_t x = 0; x < MATRIX_WIDTH; x++)
   {
      uint8_t height = band_to_height(bands_norm[x]);

      for (uint8_t h = 0; h < height; h++)
      {
         uint8_t y = (uint8_t)(MATRIX_HEIGHT - 1U - h);
         uint32_t idx = matrix_xy_to_index(x, y);

         // Active cell color.
         esp_err_t status = led_strip_set_pixel(s_led_strip, idx, 0, EQ_PIXEL_G, 0);
         if (status != ESP_OK) return status;
      }
   }
   return ESP_OK;
}

esp_err_t led_matrix_render_eq(const float *bands_norm)
{
   if (bands_norm == NULL) return ESP_ERR_INVALID_ARG;
   if (s_led_strip == NULL) return ESP_ERR_INVALID_STATE;

   esp_err_t status = clear_matrix_frame();
   if (status != ESP_OK) return status;

   status = draw_eq_columns(bands_norm);
   if (status != ESP_OK) return status;

   return led_strip_refresh(s_led_strip);
}

esp_err_t led_matrix_renderer_init(led_strip_handle_t strip)
{
   if (strip == NULL) return ESP_ERR_INVALID_ARG;
   s_led_strip = strip;
   return ESP_OK;
}
