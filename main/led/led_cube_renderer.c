#include "led_cube_renderer.h"
#include "led_cube.h"

// Default cube configuration
static led_cube_eq_config_t s_eq_config = 
{
   .low_levels = 3,
   .mid_levels = 3,
   .high_levels = 2,

   .low_color = {0, 255, 0},
   .mid_color = {255, 255, 0},
   .high_color = {255, 0, 0}
};

// Set equalizer behavior (frequency band levels and associated colors)
esp_err_t led_cube_renderer_set_eq_config(
   const led_cube_eq_config_t *config
)
{
   if (config == NULL) return ESP_ERR_INVALID_ARG;

   size_t total_levels = 
      config->low_levels +
      config->mid_levels +
      config->high_levels;

   // Make sure total levels matches 8x8x8 form factor
   if (total_levels != LED_CUBE_SIZE) return ESP_ERR_INVALID_ARG;

   s_eq_config = *config;

   return ESP_OK;
}

// Takes a z level (PCB layer) and returns configured color for that level
static led_cube_color_t eq_color_for_level(size_t z)
{
   if (z < s_eq_config.low_levels) return s_eq_config.low_color;
   if (z < (s_eq_config.low_levels + s_eq_config.mid_levels)) return s_eq_config.mid_color;
   return s_eq_config.high_color;
}

static size_t eq_band_to_height(float normalized_band_level)
{
   // Clamp normalized DSP output to expected [0.0, 1.0] range
   if (normalized_band_level < 0.0f) normalized_band_level = 0.0f;
   if (normalized_band_level > 1.0f) normalized_band_level = 1.0f;

   // Convert normalized magnitude into 0..8 active Z levels
   size_t height = (size_t)(
      normalized_band_level * (float)LED_CUBE_SIZE + 0.5f
   );

   if (height > LED_CUBE_SIZE) height = LED_CUBE_SIZE;

   return height;
}

esp_err_t led_cube_render_eq(const float *bands_norm)
{
   if (bands_norm == NULL) return ESP_ERR_INVALID_ARG;

   esp_err_t status = led_cube_clear();
   if (status != ESP_OK) return status;

   for (size_t x = 0; x < LED_CUBE_SIZE; x++)
   {
      size_t height = eq_band_to_height(bands_norm[x]);

      for (size_t z = 0; z < height; z++)
      {
         led_cube_color_t color = eq_color_for_level(z);

         // Repeat each X/Z EQ position across the full Y depth
         for (size_t y = 0; y < LED_CUBE_SIZE; y++)
         {
            status = led_cube_set_voxel(
               x,
               y,
               z,
               color.red,
               color.green,
               color.blue
            );

            if (status != ESP_OK) return status;
         }
      }
   }

   return led_cube_show();
}