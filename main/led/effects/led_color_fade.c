#include "led_color_fade.h"

// Initialize a new RGB color transition
esp_err_t led_color_fade_init(
   led_color_fade_t *fade,
   led_cube_color_t start_color,
   uint32_t transition_steps
)
{
   if (fade == NULL) return ESP_ERR_INVALID_ARG;
   if (transition_steps == 0U) return ESP_ERR_INVALID_ARG;

   // Begin with no color difference between the start and target
   fade->start_color = start_color;
   fade->target_color = start_color;
   fade->current_color = start_color;

   // Store the requested transition length and begin at the first step
   fade->transition_steps = transition_steps;
   fade->current_step = 0U;

   return ESP_OK;
}

// Begin a new transition from the current color to a new target color
esp_err_t led_color_fade_set_target(
   led_color_fade_t *fade,
   led_cube_color_t target_color
)
{
   if (fade == NULL) return ESP_ERR_INVALID_ARG;

   // Use the currently displayed color as the start of the new transition
   fade->start_color = fade->current_color;
   fade->target_color = target_color;

   // Restart interpolation from the beginning of the new transition
   fade->current_step = 0U;

   return ESP_OK;
}

// Advance the RGB transition by one animation step
esp_err_t led_color_fade_step(
   led_color_fade_t *fade
)
{
   if (fade == NULL) return ESP_ERR_INVALID_ARG;
   if (fade->transition_steps == 0U) return ESP_ERR_INVALID_STATE;

   // Stop advancing once the target color has been reached
   if (fade->current_step >= fade->transition_steps)
   {
      fade->current_color = fade->target_color;
      return ESP_OK;
   }

   fade->current_step++;

   // Calculate transition progress from 0.0 to 1.0
   float progress =
      (float)fade->current_step /
      (float)fade->transition_steps;

   // Interpolate each RGB channel independently from start to target.
   // (target - start) determines direction automatically:
   // positive increases the channel, negative decreases it, and zero leaves it unchanged.
   fade->current_color.red = (uint8_t)(
      (float)fade->start_color.red +
      ((float)fade->target_color.red - (float)fade->start_color.red) * progress
   );

   fade->current_color.green = (uint8_t)(
      (float)fade->start_color.green +
      ((float)fade->target_color.green - (float)fade->start_color.green) * progress
   );

   fade->current_color.blue = (uint8_t)(
      (float)fade->start_color.blue +
      ((float)fade->target_color.blue - (float)fade->start_color.blue) * progress
   );

   return ESP_OK;
}

// Determine whether the current RGB transition has finished
bool led_color_fade_is_complete(
   const led_color_fade_t *fade
)
{
   if (fade == NULL) return false;

   return fade->current_step >= fade->transition_steps;
}