#ifndef LED_MATRIX_RENDERER_H
#define LED_MATRIX_RENDERER_H

#include <stdint.h>

#include "esp_err.h"
#include "led_strip.h"

esp_err_t led_matrix_renderer_init(led_strip_handle_t strip);
esp_err_t led_matrix_render_eq(const float *bands_norm);

#endif
