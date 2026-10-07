#include "led_cube.h"

#include <stdbool.h>

// One independent WS2812 strip handle for each physical Z layer (one PCB worth)
static led_strip_handle_t s_layer_strips[LED_CUBE_SIZE] = {0};

static bool s_initialized = false;

esp_err_t led_cube_init(
   const led_strip_handle_t *layer_strips,
   size_t layer_count
)
{
   if (layer_strips == NULL ||
      layer_count != LED_CUBE_SIZE) return ESP_ERR_INVALID_ARG;

   // Validate and store one strip handle for each physical PCB layer
   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      if (layer_strips[layer] == NULL) return ESP_ERR_INVALID_ARG;
      s_layer_strips[layer] = layer_strips[layer];
   }

   s_initialized = true;
   return ESP_OK;
}

esp_err_t led_cube_clear(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      for (size_t pixel_index = 0; pixel_index < LED_CUBE_LAYER_PIXELS; pixel_index++)
      {
         esp_err_t status = led_strip_set_pixel(
            s_layer_strips[layer],
            pixel_index,
            0,
            0,
            0   
         );
         if (status != ESP_OK) return status;
      }
   }
   return ESP_OK;
}

// Sets a particular color of a single voxel in the cube
esp_err_t led_cube_set_voxel(
   size_t x,
   size_t y,
   size_t z,
   uint8_t red,
   uint8_t green,
   uint8_t blue
)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   size_t layer = 0;
   size_t pixel_index = 0;

   esp_err_t status = led_cube_map_voxel(x, y, z, &layer, &pixel_index);
   if (status != ESP_OK) return status;

   // Set the mapped pixel on the physical PCB selected by Z
   return led_strip_set_pixel(
      s_layer_strips[layer],
      pixel_index,
      red,
      green,
      blue
   );
}

esp_err_t led_cube_show(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      esp_err_t status = led_strip_refresh(s_layer_strips[layer]);
      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}