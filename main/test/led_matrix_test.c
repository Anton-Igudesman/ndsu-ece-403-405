#include "led_matrix_test.h"

#include "led/led_cube.h"
#include "led/led_cube_rmt.h"

#define MATRIX_SIZE 8U
#define MATRIX_TEST_LAYER 0U

esp_err_t led_matrix_test_render_eq(const float *bands_norm)
{
   if (bands_norm == NULL) return ESP_ERR_INVALID_ARG;

   led_cube_color_t pixels[LED_CUBE_LAYER_PIXELS] = {0};

   for (size_t x = 0; x < MATRIX_SIZE; x++)
   {
      float level = bands_norm[x];

      if (level < 0.0f) level = 0.0f;
      if (level > 1.0f) level = 1.0f;

      size_t height = (size_t)(
         level * (float)MATRIX_SIZE + 0.5f
      );

      if (height > MATRIX_SIZE) height = MATRIX_SIZE;

      for (size_t y = 0; y < height; y++)
      {
         // Proven physical matrix mapping:
         // columns are sequential and each runs bottom -> top
         size_t pixel_index = (x * MATRIX_SIZE) + y;

         pixels[pixel_index].red = 0;
         pixels[pixel_index].green = 32;
         pixels[pixel_index].blue = 0;
      }
   }

   return led_cube_rmt_test_matrix(pixels);
}

esp_err_t led_matrix_test_walk(void)
{
   static size_t pixel = 0;

   esp_err_t status = led_cube_rmt_test_pixel(pixel);

   if (status != ESP_OK) return status;

   pixel++;

   if (pixel >= LED_CUBE_LAYER_PIXELS)
   {
      pixel = 0;
   }

   return ESP_OK;
}