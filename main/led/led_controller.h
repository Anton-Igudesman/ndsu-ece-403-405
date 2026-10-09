#pragma once

#include "esp_err.h"
#include "led_protocol.h"

// Initialize the LED controller with its starting mode
esp_err_t led_controller_init(
   led_mode_t initial_mode
);

// Change the active LED mode
esp_err_t led_controller_set_mode(
   led_mode_t mode
);

// Return the currently active LED mode
led_mode_t led_controller_get_mode(void);