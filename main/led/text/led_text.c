#include "led_text.h"

// Default configuration for scrolling text effect
static led_text_config_t s_text_config = {
   .message = "HELLO WORLD",
   .default_color = {
      .red = 0,
      .green = 0,
      .blue = 101
   },
   .scroll_interval_ms = 150
};

// Initialize each character with default RGB color
void led_text_init(void)
{
   for (size_t i = 0; i < LED_TEXT_MAX_LENGTH; i++)
      s_text_config.character_colors[i] = s_text_config.default_color;
}

// Return the current text configuration
const led_text_config_t *led_text_get_config(void)
{
   return &s_text_config;
}