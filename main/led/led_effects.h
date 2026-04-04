#ifndef LED_EFFECTS_H
#define LED_EFFECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "led_strip.h"
#include "esp_err.h"

// Enum state for basic RGB color levels 
typedef enum
{
   LED_COLOR_RED,
   LED_COLOR_GREEN,
   LED_COLOR_BLUE
} led_color_t;

// Initialize LED state
esp_err_t led_effects_init(led_strip_handle_t strip, uint32_t index);

esp_err_t led_effects_set_color_level(led_color_t color, bool level);
esp_err_t led_effects_fade_step(void);
esp_err_t led_effects_breathe_step(
   led_color_t color,
   uint8_t min_value,
   uint8_t max_value);
#endif