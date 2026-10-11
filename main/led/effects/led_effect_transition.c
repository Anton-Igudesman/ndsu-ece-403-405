#include "led_effect_transition.h"
#include "led/led_cube.h"
#include "led_effects.h"

#include <stdint.h>
#include <stddef.h>

#define LED_TRANSITION_PHASE_UPDATES 30U

typedef enum
{
   TRANSITION_CORNERS_GREEN,
   TRANSITION_OFF_1,
   TRANSITION_CORNERS_PURPLE,
   TRANSITION_OFF_2,
   TRANSITION_COMPLETE
} transition_phase_t;

// -------------------------------
// --- Module Scope Definitions --
// -------------------------------

static transition_phase_t s_transition_phase = TRANSITION_CORNERS_GREEN;
static uint32_t s_phase_update_count = 0U;

// LED color definitions
static const led_cube_color_t TRANSITION_GREEN =
{
   .red = 0U,
   .green = LED_EFFECT_FILL_MAX_BRIGHTNESS,
   .blue = 0U
};

static const led_cube_color_t TRANSITION_PURPLE =
{
   .red = LED_EFFECT_FILL_MAX_BRIGHTNESS,
   .green = 0U,
   .blue = LED_EFFECT_FILL_MAX_BRIGHTNESS
};

// ----------------------------------------
// ----- Private Function Definitions -----
// ----------------------------------------

/* 
   Helper for transition animation renders blinking corners of cube

   Generated coordinates: (8 corners)
   (0,0,0)   (7,0,0)
   (0,7,0)   (7,7,0)

   (0,0,7)   (7,0,7)
   (0,7,7)   (7,7,7)
*/
static esp_err_t render_corners(led_cube_color_t color)
{
   for (size_t x = 0; x < LED_CUBE_SIZE; x += LED_CUBE_SIZE - 1U)
   {
      for (size_t y = 0; y < LED_CUBE_SIZE; y += LED_CUBE_SIZE - 1U)
      {
         for (size_t z = 0; z < LED_CUBE_SIZE; z += LED_CUBE_SIZE - 1U)
         {
            esp_err_t status = led_cube_set_voxel(
               x,
               y,
               z,
               color.red,
               color.green,
               color.blue
            );

            if (status != ESP_OK) return status;
         }
      }
   }

   return ESP_OK;
}

/* 
   Render center cube of transition animation
   Center 2x2x2 cube coordinates:
   (3,3,3)  (3,3,4)
   (3,4,3)  (3,4,4)
   (4,3,3)  (4,3,4)
   (4,4,3)  (4,4,4)
*/
static esp_err_t render_center(led_cube_color_t color)
{
   for (size_t x = 3U; x <= 4U; x++)
   {
      for (size_t y = 3U; y <= 4U; y++)
      {
         for (size_t z = 3U; z <= 4U; z++)
         {
            esp_err_t status = led_cube_set_voxel(
               x,
               y,
               z,
               color.red,
               color.green,
               color.blue
            );

            if (status != ESP_OK) return status;
         }
      }
   }

   return ESP_OK;
}

// ----------------------------------------
// ------ Public Function Definitions -----
// ----------------------------------------

esp_err_t led_effect_transition_init(void)
{
   s_transition_phase = TRANSITION_CORNERS_GREEN;
   s_phase_update_count = 0U;

   return ESP_OK;
}

esp_err_t led_effect_transition_step(bool *complete)
{
   if (complete == NULL) return ESP_ERR_INVALID_ARG;

   *complete = false;

   esp_err_t status = ESP_OK;

   // Render current transition phase
   switch (s_transition_phase)
   {
      case TRANSITION_CORNERS_GREEN:
         status = led_cube_clear();
         if (status != ESP_OK) return status;

         status = render_corners(TRANSITION_GREEN);
         if (status != ESP_OK) return status;

         status = render_center(TRANSITION_PURPLE);
         if (status != ESP_OK) return status;

         break;

      case TRANSITION_OFF_1:
         status = led_cube_clear();
         if (status != ESP_OK) return status;
         break;

      case TRANSITION_CORNERS_PURPLE:
         status = led_cube_clear();
         if (status != ESP_OK) return status;

         status = render_corners(TRANSITION_PURPLE);
         if (status != ESP_OK) return status;

         status = render_center(TRANSITION_GREEN);
         if (status != ESP_OK) return status;

         break;

      case TRANSITION_OFF_2:
         status = led_cube_clear();
         if (status != ESP_OK) return status;
         break;

      case TRANSITION_COMPLETE:
         *complete = true;
         return ESP_OK;

      default:
         return ESP_ERR_INVALID_STATE;
   }

   // Push the current transition phase to the physical cube
   status = led_cube_show();
   if (status != ESP_OK) return status;

   // Hold each visible phase for 30 updates = approximately 300 ms
   s_phase_update_count++;

   if (s_phase_update_count >= LED_TRANSITION_PHASE_UPDATES)
   {
      s_phase_update_count = 0U;

      switch (s_transition_phase)
      {
         case TRANSITION_CORNERS_GREEN:
            s_transition_phase = TRANSITION_OFF_1;
            break;

         case TRANSITION_OFF_1:
            s_transition_phase = TRANSITION_CORNERS_PURPLE;
            break;

         case TRANSITION_CORNERS_PURPLE:
            s_transition_phase = TRANSITION_OFF_2;
            break;

         case TRANSITION_OFF_2:
            s_transition_phase = TRANSITION_COMPLETE;
            break;

         default:
            return ESP_ERR_INVALID_STATE;
      }
   }

   return ESP_OK;
}