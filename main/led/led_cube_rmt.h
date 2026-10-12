#ifndef LED_CUBE_RMT_H
#define LED_CUBE_RMT_H

#include <stddef.h>

#include "esp_err.h"
#include "led_cube.h"

// Number of RMT transmit channels used to drive eight physical cube layers
#define LED_CUBE_RMT_CHANNEL_COUNT 4U

esp_err_t led_cube_rmt_init(
   const int *layer_gpios,
   size_t layer_count
);

esp_err_t led_cube_rmt_show(
   const led_cube_color_t layer_buffer
      [LED_CUBE_SIZE]
      [LED_CUBE_LAYER_PIXELS]
);

esp_err_t led_cube_rmt_test_pixel(size_t pixel_index);

esp_err_t led_cube_rmt_test_matrix(
   const led_cube_color_t *pixels
);

#endif