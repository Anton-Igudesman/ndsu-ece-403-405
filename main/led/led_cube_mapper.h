#pragma once

#include <stddef.h>
#include "esp_err.h"

// LED cube dimension - consists of 8 identical 8x8 PCBs stacked vertically
#define LED_CUBE_SIZE 8U
#define LED_CUBE_LAYER_PIXELS 64U

/*
   Logical cube coordinate system:

      X = left -> right
      Y = front -> back
      Z = bottom -> top

   Each Z coordinate selects one physical 8x8 PCB layer.

   Within that PCB, X and Y select a logical LED position.
   The physical WS2812 chain is wired in a serpentine pattern,
   so the mapper converts (x, y) into the correct pixel index.

   This keeps physical LED ordering separate from effects,
   DSP visualization, and future Python control.
*/

esp_err_t led_cube_map_voxel(
   size_t x,
   size_t y,
   size_t z,
   size_t *layer_out,
   size_t *pixel_index_out
);