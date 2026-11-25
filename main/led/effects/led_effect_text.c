#include "led_effect_text.h"
#include "led_cube.h"
#include "text/led_font3x5.h"
#include "text/led_text.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_timer.h"

/*
   The cube has four vertical faces, each 8 columns wide.
   The four corner columns are shared between adjacent faces,
   so each corner is counted only once.

   (4 * 8) - 4 = 28 unique perimeter columns.
*/
#define LED_TEXT_DISPLAY_WIDTH 28

// 1 = sing 8x8 matrix, 0 = full cube perimeter
#define LED_TEXT_TEST_MODE 1
#define LED_TEXT_TEST_WIDTH 8

// Add one blank bolumn between adjacent characters
#define LED_TEXT_CHARACTER_SPACING 1
#define LED_TEXT_CHARACTER_ADVANCE (LED_FONT_3x5_WIDTH + LED_TEXT_CHARACTER_SPACING)

// Runtime state for scrolling text effect
static int32_t s_scroll_offset = 0;
static int64_t s_last_scroll_time_us = 0;

// Reset scroll position and optionally start the timing clock
esp_err_t led_effect_text_init(bool start)
{
   s_scroll_offset = 0;

   // If start parameter is passed we get current time
   s_last_scroll_time_us = start ? esp_timer_get_time() : 0;

   return ESP_OK;
}

// Map a text pixel to its physical cube coordinates
static esp_err_t led_text_map_pixel(
   size_t column,
   size_t row,
   size_t *x,
   size_t *y,
   size_t *z
)
{
   if (!x || !y || !z || row >= LED_FONT_3x5_HEIGHT) return ESP_ERR_INVALID_ARG;

#if LED_TEXT_TEST_MODE
// Single matrix: X/Y plane on layer 0
   if (column >= LED_TEXT_TEST_WIDTH) return ESP_ERR_INVALID_ARG;

   *x = column;
   *y = row + 1;
   *z = 0;
#else
// Four vertical faces, traversed clockwise from (0,0)
   if (column >= LED_TEXT_DISPLAY_WIDTH) return ESP_ERR_INVALID_ARG;

   // Front face: columns 0-7
   // Move L -> R (x:0 -> 7)
   if (column < 8)
   {
      *x = column;
      *y = 0;
   }

   // Right face: columns 8-14
   // X stays at 7; subtract 7 to move Y from 1 -> 7
   else if (column < 15)
   {
      *x = 7;
      *y = column - 7;
   }

   // Back face: columns 15-21
   // Y stays at 7; subtract from 21 to move x from 6 -> 0
   else if (column < 22)
   {
      *x = 21 - column;
      *y = 7;
   }

   // Left face: columns 22-27
   // X stays at 0; subtract from 28 to move Y from 6 -> 1
   else
   {
      *x = 0;
      *y = 28 - column;
   }

   // Map the five glyph rows onto Z layers 5 though 1
   // Row 0 is top of the letter; row 4 is the bottom
   *z = 5 - row;
#endif

   return ESP_OK;
}

// Map an illuminated glyph pixel and assign its RGB color
static esp_err_t led_text_set_pixel(
   size_t column,
   size_t row,
   led_cube_color_t color
)
{
   size_t x, y, z;

   esp_err_t status = led_text_map_pixel(column, row, &x, &y, &z);
   if (status != ESP_OK) return status;

   return led_cube_set_voxel(x, y, z, color.red, color.green, color.blue);
}

// Draw one glyph at its current horizontal scroll position
static esp_err_t led_text_draw_glyph(
   const led_font_3x5_glyph_t *glyph,
   int32_t start_column,
   led_cube_color_t color
)
{
   if (!glyph) return ESP_ERR_INVALID_ARG;

   const int32_t display_width = LED_TEXT_TEST_MODE
      ? LED_TEXT_TEST_WIDTH
      : LED_TEXT_DISPLAY_WIDTH;

   for (size_t row = 0; row < LED_FONT_3x5_HEIGHT; row++)
   {
      for (size_t col = 0; col < LED_FONT_3x5_WIDTH; col++)
      {
         if (!glyph->glyph[row][col]) continue; // voxel is not lit

         // Apply horizontal position and skip pixels outside the display
         int32_t column = start_column + (int32_t)col;
         if (column < 0 || column >= display_width) continue;

         esp_err_t status = led_text_set_pixel((size_t)column, row, color);
         if (status != ESP_OK) return status;
      }
   }

   return ESP_OK;
}

// Draw the message from R -> L using current scroll offset
static esp_err_t led_text_draw_message(const led_text_config_t *config)
{
   if (config == NULL) return ESP_ERR_INVALID_ARG;

   const int32_t display_width = LED_TEXT_TEST_MODE
      ? LED_TEXT_TEST_WIDTH
      : LED_TEXT_DISPLAY_WIDTH;

   for (size_t i = 0; config->message[i] != '\0'; i++)
   {
      const led_font_3x5_glyph_t *glyph =
         led_font_3x5_get_glyph(config->message[i]);

      if (!glyph) return ESP_ERR_INVALID_ARG;

      // Start beyond the right edge and move left as offset increases
      // Each character occupies three columns with a blank column in between
      int32_t start_column = display_width - s_scroll_offset
         + (int32_t)(i * LED_TEXT_CHARACTER_ADVANCE);

      esp_err_t status = led_text_draw_glyph(glyph, start_column, config->character_colors[i]);
      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Advance scrolling text and display the current frame
esp_err_t led_effect_text_step(void)
{
   led_text_config_t config;
   esp_err_t status = led_text_get_config(&config);
   if (status != ESP_OK) return status;

   int64_t now = esp_timer_get_time();

   // Wait until the configured interval has elapsed
   if (now - s_last_scroll_time_us < (int64_t)config.scroll_interval_ms * 1000) return ESP_OK;
   s_last_scroll_time_us = now;

   const int32_t display_width = LED_TEXT_TEST_MODE
      ? LED_TEXT_TEST_WIDTH
      : LED_TEXT_DISPLAY_WIDTH;

   // Message width plus display width allows full entry and exit
   int32_t message_width = (int32_t)strlen(config.message) 
      * LED_TEXT_CHARACTER_ADVANCE - LED_TEXT_CHARACTER_SPACING;
   int32_t max_scroll = message_width + display_width;

   // Advance one column, restarting after the message exits
   s_scroll_offset++;
   if (s_scroll_offset > max_scroll) s_scroll_offset = 0;

   // Clear the previous frame before drawing the new position
   status = led_cube_clear();
   if (status != ESP_OK) return status;

   status = led_text_draw_message(&config);
   if (status != ESP_OK) return status;

   return led_cube_show();
}