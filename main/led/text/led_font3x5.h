#pragma once

#include <stdint.h>

#define LED_FONT_3x5_WIDTH 3
#define LED_FONT_3x5_HEIGHT 5

// Each glyph contains three columns with 5 bits per column
typedef struct
{
   uint8_t glyph[LED_FONT_3x5_HEIGHT][LED_FONT_3x5_WIDTH];
} led_font_3x5_glyph_t;

const led_font_3x5_glyph_t *led_font_3x5_get_glyph(char character);