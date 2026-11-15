#ifndef LED_MATRIX_TEST_H
#define LED_MATRIX_TEST_H

#include "esp_err.h"

esp_err_t led_matrix_test_render_eq(const float *bands_norm);
esp_err_t led_matrix_test_walk(void);

// Cycle through programmed LED effects, excluding Audio EQ
esp_err_t led_matrix_test_cycle_effects(void);

#endif