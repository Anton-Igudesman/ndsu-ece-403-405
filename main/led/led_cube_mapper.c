#include "led_cube_mapper.h"

esp_err_t led_cube_map_voxel(
   size_t x,
   size_t y,
   size_t z,
   size_t *layer_out,
   size_t *pixel_index_out
)
{
   // Validate output pointers before attempting coordinate mapping
   if (layer_out == NULL || pixel_index_out == NULL) return ESP_ERR_INVALID_ARG;

   // Coordinates must be within logical 8x8x8 cube
   if (x >= LED_CUBE_SIZE ||
      y >= LED_CUBE_SIZE ||
      z >= LED_CUBE_SIZE) return ESP_ERR_INVALID_ARG;

   // Each Z coordinate directly selects one physical PCB layer
   *layer_out = z;

   /* Convert logical X/Y position to physical serpentine LED index

      PCB orientation:
         - LED 1 is at (0, 0) in top left corner
         - X increases from left -> right in first row
         - Y increases from top -> bottom

      PCB data chain snakes along X:
         - Even Y rows: x = 0 -> x = 7
         - Odd y positions: x = 7 -> x = 0

      Logical coordinates are unchanged by physical direction of the data chain
   */

   if ((y % 2U) == 0U) *pixel_index_out = (y * LED_CUBE_SIZE) + x;
   else *pixel_index_out = (y * LED_CUBE_SIZE) + (LED_CUBE_SIZE - 1U - x);

   return ESP_OK;

}