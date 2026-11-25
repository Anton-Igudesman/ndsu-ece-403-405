#include "led_text.h"
#include "led_font3x5.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define DEFAULT_MESSAGE "HELLO WORLD"
#define DEFAULT_COLOR {0, 0, 101} // RGB
#define DEFAULT_SCROLL_INTERVAL_MS 150

// Default configuration for scrolling text effect
static const led_text_config_t s_default_text_config = {
   .message = DEFAULT_MESSAGE,
   .default_color = DEFAULT_COLOR,
   .character_colors = {
      [0 ... LED_TEXT_MAX_LENGTH - 1] = DEFAULT_COLOR
   },
   .scroll_interval_ms = DEFAULT_SCROLL_INTERVAL_MS,
   .style = LED_TEXT_STYLE_STANDARD,

   // Default vertical weave configuration
   .weave = {
      .pattern = LED_TEXT_WEAVE_SINE,
      .amplitude = 3,
      .wavelength = 6,
      .interval_ms = 150,
      .allow_clipping = true,
      .custom_pattern = {0, 1, 3, 1, 0, -1, -3, -1},
      .custom_pattern_length = 8
   },

   // Default moving face configuration
   .depth = {
      .mode = LED_TEXT_DEPTH_BOUNCE,
      .fixed_depth = 0,
      .min_depth = 0,
      .max_depth = 7,
      .interval_ms = 150
   }
};

// Active scrolling text configuration
static led_text_config_t s_text_config;

// Protect shared text configuration from concurrent task access
static SemaphoreHandle_t s_text_mutex = NULL;

// Initialize or reset complete text configuration
esp_err_t led_text_init(void)
{
   if (s_text_mutex == NULL)
   {
      s_text_mutex = xSemaphoreCreateMutex();
      if (s_text_mutex == NULL) return ESP_ERR_NO_MEM;
   }

   // Wait as long as necessary for mutex to become available
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
   
   s_text_config = s_default_text_config;
   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Return the current text configuration
esp_err_t led_text_get_config(led_text_config_t *config)
{
   if (config == NULL) return ESP_ERR_INVALID_ARG;
   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;

   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   *config = s_text_config;
   xSemaphoreGive(s_text_mutex);
   
   return ESP_OK;
}

/*
   Reset character colors without acquiring the mutex.
   The caller must already hold s_text_mutex to avoid
   locking the same non-recursive mutex twice.
*/
static void reset_character_colors(void)
{
   for (size_t i = 0; i < LED_TEXT_MAX_LENGTH; i++)
      s_text_config.character_colors[i] = s_default_text_config.default_color;
}

// Reset character colors to factory default
esp_err_t led_text_reset_colors(void)
{
   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   reset_character_colors();

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Replace the message and reset character colors
esp_err_t led_text_set_message(const char *message)
{
   if (!message) return ESP_ERR_INVALID_ARG;

   size_t length = strnlen(message, LED_TEXT_MAX_LENGTH + 1);
   if (length == 0 || length > LED_TEXT_MAX_LENGTH) return ESP_ERR_INVALID_ARG;

   // Validate characters before changing message
   for (size_t i = 0; i < length; i++)
      if (!led_font_3x5_get_glyph(message[i])) return ESP_ERR_INVALID_ARG;

   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   // Update message and colors together under the same lock
   memcpy(s_text_config.message, message, length + 1);

   // Reset character colors to default
   reset_character_colors();

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Update default color without modifying existing character colors
esp_err_t led_text_set_default_color(led_cube_color_t color)
{
   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   s_text_config.default_color = color;

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Assign an RGB color to a specific character index
esp_err_t led_text_set_character_color(size_t index, led_cube_color_t color)
{
   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   if (index >= strlen(s_text_config.message))
   {
      xSemaphoreGive(s_text_mutex);
      return ESP_ERR_INVALID_ARG;
   } 

   s_text_config.character_colors[index] = color;

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Select the scrolling text animation style
esp_err_t led_text_set_style(led_text_style_t style)
{
   // Reject unsupported animation styles
   switch (style)
   {
      case LED_TEXT_STYLE_STANDARD:
      case LED_TEXT_STYLE_WEAVE:
      case LED_TEXT_STYLE_MOVING_FACE:
         break;

      default:
         return ESP_ERR_INVALID_ARG; 
   }

   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   s_text_config.style = style;

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Validate and update complete vertical weaving configuration
esp_err_t led_text_set_weave_config(const led_text_weave_config_t *config)
{
   if (config == NULL) return ESP_ERR_INVALID_ARG;

   // Validate selected weave pattern
   switch(config->pattern)
   {
      case LED_TEXT_WEAVE_SINE:
      case LED_TEXT_WEAVE_ZIGZAG:
      case LED_TEXT_WEAVE_ALTERNATING:
      case LED_TEXT_WEAVE_STAIRCASE:
      case LED_TEXT_WEAVE_CUSTOM:
         break;
      
      default:
         return ESP_ERR_INVALID_ARG;
   }

   // Validate amplitude, wavelength, and movement interval
   if (config->amplitude > 5 ||
      config->wavelength < 2 || config->wavelength > 12 ||
      config->interval_ms < 50 || config->interval_ms > 2000) return ESP_ERR_INVALID_ARG;

   // Validate custom pattern length
   if (config->custom_pattern_length == 0 ||
      config->custom_pattern_length > LED_TEXT_MAX_PATTERN_LENGTH) return ESP_ERR_INVALID_ARG;

   // Validate signed vertical offsets for custom pattern entry
   for (size_t i = 0; i < config->custom_pattern_length; i++)
   {
      if (config->custom_pattern[i] < -5 ||
         config->custom_pattern[i] > 5) return ESP_ERR_INVALID_ARG;
   }

   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   // Update weave setting together under one lock
   s_text_config.weave = *config;

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Validate and update moving face depth configuration
esp_err_t led_text_set_depth_config(const led_text_depth_config_t *config)
{
   if (config == NULL) return ESP_ERR_INVALID_ARG;

   // Validate selected depth movement mode
   switch (config->mode)
   {
      case LED_TEXT_DEPTH_FIXED:
      case LED_TEXT_DEPTH_BOUNCE:
         break;

      default:
         return ESP_ERR_INVALID_ARG;
   }

   // Ensure all depth positions are within physical cube
   if (config->fixed_depth >= LED_CUBE_SIZE ||
      config->min_depth >= LED_CUBE_SIZE ||
      config->max_depth >= LED_CUBE_SIZE) return ESP_ERR_INVALID_ARG;

   // Ensure bounding depth has valid movement range
   if (config->min_depth >= config->max_depth) return ESP_ERR_INVALID_ARG;
   if (config->interval_ms < 50 || config->interval_ms > 2000) return ESP_ERR_INVALID_ARG;

   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   s_text_config.depth = *config;

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}

// Change the interval between scrolling movements
esp_err_t led_text_set_scroll_interval(uint16_t interval_ms)
{
   if (interval_ms < 50 || interval_ms > 2000) return ESP_ERR_INVALID_ARG;

   if (s_text_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_text_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   s_text_config.scroll_interval_ms = interval_ms;

   xSemaphoreGive(s_text_mutex);

   return ESP_OK;
}