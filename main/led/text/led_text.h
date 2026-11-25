#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "led_cube.h"

#define LED_TEXT_MAX_LENGTH 12
#define LED_TEXT_MAX_PATTERN_LENGTH 12

// Available scrolling text animation styles
typedef enum
{
   LED_TEXT_STYLE_STANDARD,
   LED_TEXT_STYLE_WEAVE,
   LED_TEXT_STYLE_MOVING_FACE
} led_text_style_t;

// Available vertical weaving patterns
typedef enum
{
   LED_TEXT_WEAVE_SINE,
   LED_TEXT_WEAVE_ZIGZAG,
   LED_TEXT_WEAVE_ALTERNATING,
   LED_TEXT_WEAVE_STAIRCASE,
   LED_TEXT_WEAVE_CUSTOM
} led_text_weave_pattern_t;

// Depth movement behavior for the moving face
typedef enum
{
   LED_TEXT_DEPTH_FIXED,
   LED_TEXT_DEPTH_BOUNCE
} led_text_depth_mode_t;

// Configuration for vertical weaving
typedef struct
{
   led_text_weave_pattern_t pattern;
   uint8_t amplitude;
   uint8_t wavelength;
   uint16_t interval_ms;
   bool allow_clipping;

   // User defined erpeating sequence of signed vertical offsets
   int8_t custom_pattern[LED_TEXT_MAX_PATTERN_LENGTH];
   uint8_t custom_pattern_length;
} led_text_weave_config_t;

// Configuration for moving face depth
typedef struct
{
   led_text_depth_mode_t mode;
   uint8_t fixed_depth;
   uint8_t min_depth;
   uint8_t max_depth;
   uint16_t interval_ms;
} led_text_depth_config_t;

typedef struct 
{
   char message[LED_TEXT_MAX_LENGTH + 1];
   led_cube_color_t default_color;
   led_cube_color_t character_colors[LED_TEXT_MAX_LENGTH];
   uint16_t scroll_interval_ms;

   // Selected animation and independent animation configurations
   led_text_style_t style;
   led_text_weave_config_t weave;
   led_text_depth_config_t depth;
} led_text_config_t;

esp_err_t led_text_init(void);
esp_err_t led_text_get_config(led_text_config_t *config);
esp_err_t led_text_reset_colors(void);
esp_err_t led_text_set_message(const char *message);
esp_err_t led_text_set_default_color(led_cube_color_t color);
esp_err_t led_text_set_character_color(size_t index, led_cube_color_t color);
esp_err_t led_text_set_scroll_interval(uint16_t interval_ms);
esp_err_t led_text_set_style(led_text_style_t style);
esp_err_t led_text_set_weave_config(const led_text_weave_config_t *config);
esp_err_t led_text_set_depth_config(const led_text_depth_config_t *config);