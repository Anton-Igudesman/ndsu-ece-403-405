#pragma once

#include <stddef.h>
#include <stdint.h>

#include "led_cube.h"

#define LED_TEXT_MAX_LENGTH 12

typedef struct 
{
   char message[LED_TEXT_MAX_LENGTH + 1];
   led_cube_color_t default_color;
   led_cube_color_t character_colors[LED_TEXT_MAX_LENGTH];
   uint16_t scroll_interval_ms;
} led_text_config_t;

void led_text_init(void);
const led_text_config_t *led_text_get_config(void);