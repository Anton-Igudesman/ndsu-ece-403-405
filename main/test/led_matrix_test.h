#ifndef LED_MATRIX_TEST_H
#define LED_MATRIX_TEST_H

#include "esp_err.h"

esp_err_t led_matrix_test_render_eq(const float *bands_norm);
esp_err_t led_matrix_test_walk(void);

#endif