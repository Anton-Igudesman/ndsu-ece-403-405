#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "led_strip.h"
#include "led_cube_mapper.h"

/*
   Renderer for the final 8x8x8 LED cube hardware.

   The cube consists of eight independent 8x8 WS2812 PCBs.
   Each PCB has its own LED strip handle/data line and represents
   one logical Z layer.

   Coordinate system:
      X = left -> right
      Y = front -> back
      Z = bottom -> top
*/

/*
   Initialize the renderer with one strip handle for each physical
   PCB layer.

   layer_strips[0] corresponds to z=0 (bottom PCB).
   layer_strips[7] corresponds to z=7 (top PCB).
*/

esp_err_t led_cube_init(
   const led_strip_handle_t *layer_strips,
   size_t layer_count
);

/*
   Set the color of one logical voxel in the cube

   The cube mapper converts (x, y, z) into the correct physical
   PCB layer and serpentine pixel index
*/
esp_err_t led_cube_set_voxel(
   size_t x,
   size_t y,
   size_t z,
   uint8_t red,
   uint8_t green,
   uint8_t blue
);

typedef struct
{
   uint8_t red;
   uint8_t green;
   uint8_t blue;
} led_cube_color_t;

/*
   Clear all 512 logical voxels.

   Pixel data is cleared in memory for all eight physical layers.
   This does not refresh/transmit the cube.
*/
esp_err_t led_cube_clear(void);

// Display current cube frame
esp_err_t led_cube_show(void);