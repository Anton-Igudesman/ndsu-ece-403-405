#include "led_cube.h"
#include "led_cube_rmt.h"

#include <stdbool.h>
#include <string.h>

// Store RGB state for all 64 LEDs on each of the eight cube layers
static led_cube_color_t s_layer_buffer
   [LED_CUBE_SIZE]
   [LED_CUBE_LAYER_PIXELS] = {0};

static bool s_initialized = false;

// ----------------------------------------
// ----- Private Function Definitions -----
// ----------------------------------------

// Clear RGB state for all pixels across all eight cube layers
static void clear_layer_buffer(void)
{
   memset(
      s_layer_buffer,
      0,
      sizeof(s_layer_buffer)
   );
}

// ----------------------------------------
// ------ Public Function Definitions -----
// ----------------------------------------

esp_err_t led_cube_clear(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   // Clear RGB state
   clear_layer_buffer();
   
   return ESP_OK;
}

esp_err_t led_cube_init(void)
{
   s_initialized = true;

   // Initialize cube with all stored pixel states off
   esp_err_t status = led_cube_clear();
   if (status != ESP_OK)
   {
      s_initialized = false;
      return status;
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

   // Store voxel color in mapped layer and physical pixel position
   s_layer_buffer[layer][pixel_index] = (led_cube_color_t)
   {
      .red = red,
      .green = green,
      .blue = blue
   };

   return ESP_OK;
}

/*
   Display the current logical cube state through the physical RMT driver.
   RMT channel management and WS2812 transmission remain hidden from callers.
*/
esp_err_t led_cube_show(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   return led_cube_rmt_show(s_layer_buffer);
}