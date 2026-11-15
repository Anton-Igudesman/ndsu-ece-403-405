#include "led_matrix_test.h"
#include "led/led_cube.h"
#include "led/led_cube_rmt.h"
#include "led/led_controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MATRIX_SIZE 8U
#define MATRIX_TEST_LAYER 0U

// LED effects and their test durations in ms
typedef struct
{
   led_mode_t mode;
   uint32_t duration_ms;
} led_effect_test_entry_t;

static const led_effect_test_entry_t s_effect_test_sequence[] = 
{
   {LED_MODE_BLINK, 10000U},
   {LED_MODE_FADE, 16000U},
   {LED_MODE_BREATHE, 10000U},
   {LED_MODE_POCKETS, 20000U},
   {LED_MODE_OFF, 5000U}
};

// Track the active effect and its elapsed test time
static size_t s_effect_test_index = 0U;
static TickType_t s_effect_start_tick = 0;
static bool s_effect_test_started = false;
static bool s_effect_timer_started = false;

esp_err_t led_matrix_test_render_eq(const float *bands_norm)
{
   if (bands_norm == NULL) return ESP_ERR_INVALID_ARG;

   led_cube_color_t pixels[LED_CUBE_LAYER_PIXELS] = {0};

   for (size_t x = 0; x < MATRIX_SIZE; x++)
   {
      float level = bands_norm[x];

      if (level < 0.0f) level = 0.0f;
      if (level > 1.0f) level = 1.0f;

      size_t height = (size_t)(
         level * (float)MATRIX_SIZE + 0.5f
      );

      if (height > MATRIX_SIZE) height = MATRIX_SIZE;

      for (size_t y = 0; y < height; y++)
      {
         // Proven physical matrix mapping:
         // columns are sequential and each runs bottom -> top
         size_t pixel_index = (x * MATRIX_SIZE) + y;

         pixels[pixel_index].red = 0;
         pixels[pixel_index].green = 32;
         pixels[pixel_index].blue = 0;
      }
   }

   return led_cube_rmt_test_matrix(pixels);
}

esp_err_t led_matrix_test_walk(void)
{
   static size_t pixel = 0;

   esp_err_t status = led_cube_rmt_test_pixel(pixel);

   if (status != ESP_OK) return status;
   pixel++;

   if (pixel >= LED_CUBE_LAYER_PIXELS) pixel = 0;
   

   return ESP_OK;
}

// Cycle through the configured LED effects
esp_err_t led_matrix_test_cycle_effects(void)
{
   esp_err_t status;
   // Start the first effect when test begins
   if (!s_effect_test_started)
   {
      status = led_controller_set_mode(
         s_effect_test_sequence[s_effect_test_index].mode
      );

      if (status != ESP_OK) return status;

      s_effect_test_started = true;
      s_effect_timer_started = false;
   }

   // Wait until the controller finishes transitioning to requested effect
   if (led_controller_is_transitioning() ||
      led_controller_get_mode() != s_effect_test_sequence[s_effect_test_index].mode) return ESP_OK;

   // Start timing only after requested effect becomes active
   if (!s_effect_timer_started)
   {
      s_effect_start_tick = xTaskGetTickCount();
      s_effect_timer_started = true;
   } 

   // Check whether current effect has completed test duration
   TickType_t elapsed = xTaskGetTickCount() - s_effect_start_tick;

   if (elapsed >= pdMS_TO_TICKS(s_effect_test_sequence[s_effect_test_index].duration_ms))
   {
      // Advance to the next effect, wrapping around after last
      s_effect_test_index++;

      if (s_effect_test_index >= sizeof(s_effect_test_sequence) / sizeof(s_effect_test_sequence[0]))
         s_effect_test_index = 0U;

      // Request the next effect through LED controller
      status = led_controller_set_mode(
         s_effect_test_sequence[s_effect_test_index].mode
      );

      if (status != ESP_OK) return status;

      // Wait for the next mode to become active before restarting timer
      s_effect_timer_started = false;
   }

   return ESP_OK;   
}