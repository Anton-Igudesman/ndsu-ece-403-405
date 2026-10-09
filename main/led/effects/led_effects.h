#ifndef LED_EFFECTS_H
#define LED_EFFECTS_H

#include <stdint.h>
#include <stdbool.h>
#include "led_cube.h"
#include "esp_err.h"

#define LED_EFFECT_FILL_MAX_BRIGHTNESS 200U
#define LED_EFFECT_FADE_TRANSITION_STEPS 200U

// Enum state for basic RGB color levels 
typedef enum
{
   LED_COLOR_RED,
   LED_COLOR_GREEN,
   LED_COLOR_BLUE
} led_color_t;

// Effects available to the cube control layer
typedef enum
{
   LED_EFFECT_NONE = 0,
   LED_EFFECT_EQ,
   LED_EFFECT_SOLID,
   LED_EFFECT_FADE,
   LED_EFFECT_BREATHE,
   LED_EFFECT_WAVE,
   LED_EFFECT_PROGRAMMABLE
} led_effect_type_t;

// Logical direction used by effects that move through the cube
/*
   X_POS = x 0 → 7
   X_NEG = x 7 → 0

   Y_POS = y 0 → 7
   Y_NEG = y 7 → 0

   Z_POS = z 0 → 7
   Z_NEG = z 7 → 0
*/
typedef enum
{
   LED_EFFECT_DIRECTION_X_POS = 0,
   LED_EFFECT_DIRECTION_X_NEG,
   LED_EFFECT_DIRECTION_Y_POS,
   LED_EFFECT_DIRECTION_Y_NEG,
   LED_EFFECT_DIRECTION_Z_POS,
   LED_EFFECT_DIRECTION_Z_NEG
} led_effect_direction_t;

esp_err_t led_effects_set_solid_color(
   uint8_t red,
   uint8_t green,
   uint8_t blue
);

esp_err_t led_effects_fade_step(void);
esp_err_t led_effects_breathe_step(
   led_color_t color,
   uint8_t min_value,
   uint8_t max_value);
#endif