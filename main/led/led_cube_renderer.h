#pragma once

#include <stddef.h>
#include<stdint.h>

#include "esp_err.h"
#include "led_cube.h"

/*
   Render an 8-band audio spectrum into the 8x8x8 LED cube.

   EQ mapping:
      X = frequency band
      Y = depth
      Z = amplitude / height

   Each active X/Z position is repeated across the full Y depth.

   Default EQ colors are assigned by Z height:
      lower levels  = green
      middle levels = yellow
      upper levels  = red
*/

typedef struct
{
   size_t low_levels;
   size_t mid_levels;
   size_t high_levels;

   led_cube_color_t low_color;
   led_cube_color_t mid_color;
   led_cube_color_t high_color;
} led_cube_eq_config_t;

/*
   Configure EQ zone sizes and colors.

   The total number of configured levels must equal LED_CUBE_SIZE.
*/
esp_err_t led_cube_renderer_set_eq_config(
   const led_cube_eq_config_t *config
);

/*
   Render normalized 8-band spectrum data into the cube.

   X = frequency band
   Y = full cube depth
   Z = amplitude / height
*/
esp_err_t led_cube_render_eq(
   const float *bands_norm
);