#include "led_effects.h"

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
static led_strip_handle_t s_led_strip;
static uint32_t s_led_index = 0;
static fade_state_t s_led_state;
static led_color_t s_last_breathe_color = LED_COLOR_RED;

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

// Define error handling function
static esp_err_t led_effects_commit_rgb(
   uint8_t r, 
   uint8_t g, 
   uint8_t b)
{
   esp_err_t status = led_strip_set_pixel(
      s_led_strip,
      s_led_index,
      r,
      g,
      b
   );

   if (status != ESP_OK) return status; // Read early errors
   return led_strip_refresh(s_led_strip);
}

// ----- Public Function Definitions -----
esp_err_t led_effects_init(led_strip_handle_t strip, uint32_t index)
{
   if (strip == NULL) return ESP_ERR_INVALID_ARG;

   // Create module state
   s_led_strip = strip;
   s_led_index = index;
   
   // Initialize first state of LED on startup
   led_effects_initial_state(LED_COLOR_RED);

   return ESP_OK;
}

esp_err_t led_effects_set_color_level(led_color_t color, bool level)
{
   // Set all RBG levels to 0 initially
   uint8_t r = 0;
   uint8_t g = 0;
   uint8_t b = 0;

   if (level)
   {
      // Logic for setting RGB base color
      switch (color)
      {
         case LED_COLOR_RED:
            r = 255;
            break;
         
         case LED_COLOR_GREEN:
            g = 255;
            break;
         
         case LED_COLOR_BLUE:
            b = 255;
            break;
         
         default:
            return ESP_ERR_INVALID_ARG;
      }
   }

   return led_effects_commit_rgb(r, g, b);
}

esp_err_t led_effects_fade_step(void)
{

   // Fade logic depending on color state
   switch (s_led_state.phase)
   {
      case FADE_RED_TO_PURPLE:
         s_led_state.b += s_led_state.step;
         
         // When we reach saturated color, switch to next phase
         if (s_led_state.b >= 255)
         {
            s_led_state.b = 255;
            s_led_state.phase = FADE_PURPLE_TO_GREEN;
         }
         break;
      
      case FADE_PURPLE_TO_GREEN:
         s_led_state.r -= s_led_state.step;
         s_led_state.b -= s_led_state.step;
         s_led_state.g += s_led_state.step;

         if (s_led_state.r < 0) s_led_state.r = 0;
         if (s_led_state.b < 0) s_led_state.b = 0;
         if (s_led_state.g > 255) s_led_state.g = 255;

         if (
            s_led_state.r == 0 &&
            s_led_state.b == 0 &&
            s_led_state.g == 255
         ) s_led_state.phase = FADE_GREEN_TO_BLUE;
         break;

      case FADE_GREEN_TO_BLUE:
         s_led_state.b += s_led_state.step;
         s_led_state.g -= s_led_state.step;

         if (s_led_state.b > 255) s_led_state.b = 255;
         if (s_led_state.g < 0) s_led_state.g = 0;

         if (
            s_led_state.b == 255 &&
            s_led_state.g == 0
         ) s_led_state.phase = FADE_BLUE_TO_RED;
         break;

      case FADE_BLUE_TO_RED:
         s_led_state.b -= s_led_state.step;
         s_led_state.r += s_led_state.step;

         if (s_led_state.b < 0) s_led_state.b =0;
         if (s_led_state.r > 255) s_led_state.r = 255;

         if (
            s_led_state.b == 0 &&
            s_led_state.r == 255
         ) s_led_state.phase = FADE_RED_TO_PURPLE;
         break;

      default:
         return ESP_ERR_INVALID_ARG;
   }
   return led_effects_commit_rgb(
      (uint8_t)s_led_state.r,
      (uint8_t)s_led_state.g,
      (uint8_t)s_led_state.b
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

   return led_effects_commit_rgb(
      (uint8_t)s_led_state.r,
      (uint8_t)s_led_state.g,
      (uint8_t)s_led_state.b
   );
}

