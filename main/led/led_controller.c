#include "led_controller.h"
#include "led_cube_renderer.h"
#include "effects/led_effects.h"
#include "effects/led_effect_pockets.h"
#include "effects/led_effect_transition.h"

#include <stdbool.h>

// -------------------------------
// --- Module Scope Definitions --
// -------------------------------

static led_mode_t s_active_mode = LED_MODE_OFF;
static led_mode_t s_pending_mode = LED_MODE_OFF;
static bool s_initialized = false;
static bool s_transitioning = false;

// ----------------------------------------
// ------ Public Function Definitions -----
// ----------------------------------------

// Formalize the start pattern
esp_err_t led_controller_init(led_mode_t initial_mode)
{
   esp_err_t status = led_controller_set_mode(initial_mode);

   if (status != ESP_OK) return status;
   s_initialized = true;

   return ESP_OK;
}

esp_err_t led_controller_set_mode(led_mode_t mode)
{
   // Validate requested mode
   switch (mode)
   {
      case LED_MODE_BLINK:
      case LED_MODE_FADE:
      case LED_MODE_BREATHE:
      case LED_MODE_AUDIO_EQ:
      case LED_MODE_POCKETS:
      case LED_MODE_OFF:
         break;

      default:
         return ESP_ERR_INVALID_ARG;
   }

   // Initial startup selects the requested mode immediately
   if (!s_initialized)
   {
      if (mode == LED_MODE_POCKETS)
      {
         esp_err_t status = led_effect_pockets_init();
         if (status != ESP_OK) return status;
      }

      s_active_mode = mode;

      return ESP_OK;
   }

   // Ignore requests to re-select the currently active mode
   if (!s_transitioning && mode == s_active_mode) return ESP_OK;

   // If already transitioning, update the destination without restarting
   if (s_transitioning)
   {
      s_pending_mode = mode;
      return ESP_OK;
   }

   // Begin transition before activating the newly requested mode
   esp_err_t status = led_effect_transition_init();
   if (status != ESP_OK) return status;

   s_pending_mode = mode;
   s_transitioning = true;

   return ESP_OK;
}

// Return the currently active LED mode
led_mode_t led_controller_get_mode(void) {return s_active_mode;}

// Dispatch execution to currently active LED mode
esp_err_t led_controller_dispatch_active_mode(const float *bands_norm)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   if (s_transitioning)
   {
      bool transition_complete = false;

      esp_err_t status = led_effect_transition_step(
         &transition_complete
      );

      if (status != ESP_OK) return status;

      if (!transition_complete) return ESP_OK;

      // Transition finished - initialize the pending mode if required
      if (s_pending_mode == LED_MODE_POCKETS)
      {
         status = led_effect_pockets_init();
         if (status != ESP_OK) return status;
      }

      // Promote the pending mode to the active mode
      s_active_mode = s_pending_mode;
      s_pending_mode = LED_MODE_OFF;
      s_transitioning = false;

      return ESP_OK;
   }

   switch (s_active_mode)
   {
      case LED_MODE_AUDIO_EQ:
         if (bands_norm == NULL) return ESP_ERR_INVALID_ARG;
         return led_cube_render_eq(bands_norm);

      case LED_MODE_FADE:
         return led_effects_fade_step();

      case LED_MODE_BREATHE:
         return led_effects_breathe_step(
            LED_COLOR_RED,
            55U,
            255U
         );

      case LED_MODE_POCKETS:
         return led_effect_pockets_step();

      case LED_MODE_BLINK:
         return led_effects_blink_step();

      case LED_MODE_OFF:
         return led_effects_set_solid_color(
            0U,
            0U,
            0U
         );

      default:
         return ESP_ERR_INVALID_STATE;
   }
}