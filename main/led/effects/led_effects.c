#include <math.h>

#include "led_effects.h"
#include "led_color_fade.h"

// State machine for programmed color fade patterns
typedef enum
{
   FADE_RED_TO_PURPLE,
   FADE_PURPLE_TO_GREEN,
   FADE_GREEN_TO_BLUE,
   FADE_BLUE_TO_RED
} fade_phase_t;

// Struct for fading color effect
typedef struct
{
   // Use int values for step increment/decrement
   // Cast to uint8_t when committing to RGB driver
   int r; // red 
   int g; // green 
   int b; // blue 

   int delta_r; // -1, 0, +1
   int delta_g; 
   int delta_b;

   int step; // amount to change each update
   fade_phase_t phase; // while pattern
} fade_state_t;

// ----- Module Scope Definitions ------
static fade_state_t s_led_state;
static led_color_t s_last_breathe_color = LED_COLOR_RED;

// Color transition state used by full-cube fade effect
static led_color_fade_t s_fade;
static bool s_fade_initialized = false;
static fade_phase_t s_current_fade_phase = FADE_RED_TO_PURPLE;

// Initialize full-cube fade with starting color and first target
static esp_err_t led_effects_fade_init(void)
{
   led_cube_color_t start_color = 
   {
      .red = LED_EFFECT_FILL_MAX_BRIGHTNESS,
      .green = 0U,
      .blue = 0U
   };

   led_cube_color_t target_color = 
   {
      .red = LED_EFFECT_FILL_MAX_BRIGHTNESS,
      .green = 0U,
      .blue = LED_EFFECT_FILL_MAX_BRIGHTNESS
   };

   esp_err_t status = led_color_fade_init(
      &s_fade,
      start_color,
      LED_EFFECT_FADE_TRANSITION_STEPS
   );
   if (status != ESP_OK) return status;

   status = led_color_fade_set_target(
      &s_fade,
      target_color
   );
   if (status != ESP_OK) return status;

   s_current_fade_phase = FADE_RED_TO_PURPLE;
   s_fade_initialized = true;

   return ESP_OK;
}

// Control all 512 LED's at once
esp_err_t led_set_solid_color(
   uint8_t red,
   uint8_t green,
   uint8_t blue
)
{
   // Full-cube effects are capped to limit worst-case current draw
   // Limit each RGB channel for effects that can illuminate all 512 voxels
   if (red > LED_EFFECT_FILL_MAX_BRIGHTNESS) red = LED_EFFECT_FILL_MAX_BRIGHTNESS;
   if (green > LED_EFFECT_FILL_MAX_BRIGHTNESS) green = LED_EFFECT_FILL_MAX_BRIGHTNESS;
   if (blue > LED_EFFECT_FILL_MAX_BRIGHTNESS) blue = LED_EFFECT_FILL_MAX_BRIGHTNESS;

   for (size_t z = 0; z < LED_CUBE_SIZE; z++)
   {
      for (size_t y = 0; y < LED_CUBE_SIZE; y++)
      {
         for (size_t x = 0; x < LED_CUBE_SIZE; x++)
         {
            esp_err_t status = led_cube_set_voxel(
               x,
               y,
               z,
               red,
               green,
               blue
            );

            if (status != ESP_OK) return status;
         }
      }
   }

   return led_cube_show();
}

// Second argument will override
static void init_color_values(led_color_t color)
{
   switch (color)
   {
      case LED_COLOR_RED:
         s_led_state.r = 255;
         s_led_state.delta_r = -1;
         s_led_state.phase = FADE_RED_TO_PURPLE;
         break;
      
      case LED_COLOR_GREEN:
         s_led_state.g = 255;
         s_led_state.delta_g = -1;
         s_led_state.phase = FADE_GREEN_TO_BLUE;
         break;

      case LED_COLOR_BLUE:
         s_led_state.b = 255;
         s_led_state.delta_b = -1;
         s_led_state.phase = FADE_BLUE_TO_RED;
         break;

      default:
         s_led_state = (fade_state_t){0};
   }
}

static void led_effects_initial_state(led_color_t color)
{
   // Setting initial parameters for LED stae
   s_led_state = (fade_state_t){0};
   s_led_state.step = 1;
   init_color_values(color);
}
// ---------------------------------------
// ----- Public Function Definitions -----
// ---------------------------------------

esp_err_t led_effects_fade_step(void)
{

   if (!s_fade_initialized)
   {
      esp_err_t status = led_effects_fade_init();
      if (status != ESP_OK) return status;
   }

   // Advance current RGB transition
   esp_err_t status = led_color_fade_step(&s_fade);
   if (status != ESP_OK) return status;

   // Select next transition after reaching the current target
   if (led_color_fade_is_complete(&s_fade))
   {
      led_cube_color_t next_color;

      switch (s_current_fade_phase)
      {
         case FADE_RED_TO_PURPLE:
            next_color = (led_cube_color_t)
            {
               .red = 0U,
               .green = LED_EFFECT_FILL_MAX_BRIGHTNESS,
               .blue = 0U
            };
            s_current_fade_phase = FADE_PURPLE_TO_GREEN;
            break;

         case FADE_PURPLE_TO_GREEN:
            next_color = (led_cube_color_t)
            {
               .red = 0U,
               .green = 0U,
               .blue = LED_EFFECT_FILL_MAX_BRIGHTNESS
            };
            s_current_fade_phase = FADE_GREEN_TO_BLUE;
            break;

         case FADE_GREEN_TO_BLUE:
            next_color = (led_cube_color_t)
            {
               .red = LED_EFFECT_FILL_MAX_BRIGHTNESS,
               .green = 0U,
               .blue = 0U
            };
            s_current_fade_phase = FADE_BLUE_TO_RED;
            break;

         case FADE_BLUE_TO_RED:
            next_color = (led_cube_color_t)
            {
               .red = LED_EFFECT_FILL_MAX_BRIGHTNESS,
               .green = 0U,
               .blue = LED_EFFECT_FILL_MAX_BRIGHTNESS
            };
            s_current_fade_phase = FADE_RED_TO_PURPLE;
            break;

         default:
            return ESP_ERR_INVALID_STATE;
      }

      status = led_color_fade_set_target(
         &s_fade,
         next_color
      );
      if (status != ESP_OK) return status;
   }

   // Apply the current fade color to the complete cube
   return led_effects_set_solid_color(
      s_fade.current_color.red,
      s_fade.current_color.green,
      s_fade.current_color.blue
   );
}

esp_err_t led_effects_breathe_step(
   led_color_t color,
   uint8_t min_value,
   uint8_t max_value)
{
   
   if (min_value >= max_value) return ESP_ERR_INVALID_ARG;

   // Force restart on color change
   if (color != s_last_breathe_color)
   {
      s_led_state = (fade_state_t){0};
      s_led_state.step = 1;
      init_color_values(color);

      // Start selected channel at full max and decrement
      switch(color)
      {
         case LED_COLOR_RED:
            s_led_state.r = max_value;
            s_led_state.delta_r = -1;
            break;
         
         case LED_COLOR_GREEN:
            s_led_state.g = max_value;
            s_led_state.delta_g = -1;
            break;

         case LED_COLOR_BLUE:
            s_led_state.b = max_value;
            s_led_state.delta_b = -1;
            break;

         default:
            return ESP_ERR_INVALID_ARG;
      }
   }
   
   // Updating last used color
   s_last_breathe_color = color;
   
   // Local pointers to reduce color change logic
   int *active_value = NULL;
   int *active_delta = NULL;

   switch (color)
   {
      case LED_COLOR_RED:
         active_value = &s_led_state.r;
         active_delta = &s_led_state.delta_r;
         s_led_state.g = 0;
         s_led_state.b = 0;
         break;

      case LED_COLOR_GREEN:
         active_value = &s_led_state.g;
         active_delta = &s_led_state.delta_g;
         s_led_state.r = 0;
         s_led_state.b = 0;
         break;

      case LED_COLOR_BLUE:
         active_value = &s_led_state.b;
         active_delta = &s_led_state.delta_b;
         s_led_state.r = 0;
         s_led_state.g = 0;
         break;

      default:
         return ESP_ERR_INVALID_ARG;
   }

   // Make sure the active_delta is in decrement mode 
   if (*active_delta == 0) *active_delta = -1;

   int value = *active_value + ((*active_delta) * s_led_state.step);

   // Toggle between increment/decrement when reaching limits
   if (value >= (int)max_value)
   {
      value = (int)max_value;
      *active_delta = -1;
   }

   else if (value <= (int)min_value)
   {
      value = (int)min_value;
      *active_delta = +1;
   }

   *active_value = value;

   // Apply the current breathe color to the complete cube
   return led_effects_set_solid_color(
      (uint8_t)s_led_state.r,
      (uint8_t)s_led_state.g,
      (uint8_t)s_led_state.b
   );
}
