#include <math.h>

#include "led_effects.h"
#include "led_color_fade.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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

// ---- Forward declarations
// Shared proportional RGB brightness scaling for Blink and Fade
static uint8_t led_blink_scale_channel(
   uint8_t channel,
   uint8_t brightness_percent
);

// -------------------------------------
// ----- Module Scope Definitions ------
// -------------------------------------

static fade_state_t s_led_state;
static led_color_t s_last_breathe_color = LED_COLOR_RED;
static led_blink_config_t s_blink_layers[LED_CUBE_SIZE] = {0};

// Runtime timing state for each Blink layer
typedef struct
{
   TickType_t last_toggle_tick;
   bool is_on;
} led_blink_layer_state_t;

static led_blink_layer_state_t s_blink_layer_states[LED_CUBE_SIZE] = {0};

// Protect shared Blink configuration and timing state
static SemaphoreHandle_t s_blink_mutex = NULL;

// Saved Fade configuration for each physical cube layer
static led_fade_config_t s_fade_layers[LED_CUBE_SIZE] = {0};

// Runtime interpolation and timing state for each Fade layer
typedef struct
{
   led_color_fade_t fade;
   uint8_t target_color_index;
} led_fade_layer_state_t;

static led_fade_layer_state_t s_fade_layer_states[LED_CUBE_SIZE] = {0};
static SemaphoreHandle_t s_fade_mutex = NULL;

// Initialize full-cube fade with starting color and first target
esp_err_t led_effects_fade_init(void)
{
   if (s_fade_mutex == NULL)
   {
      s_fade_mutex = xSemaphoreCreateMutex();
      if (s_fade_mutex == NULL) return ESP_ERR_NO_MEM;
   }

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      // Default to a three-color RGB sequence
      s_fade_layers[layer] = (led_fade_config_t)
      {
         .colors = 
         {
            {255U, 0U, 0U},
            {0U, 255U, 0U},
            {0U, 0U, 255U}
         },
         .color_count = 3U,
         .brightness_percent = 100U,
         .transition_steps = LED_EFFECT_FADE_TRANSITION_STEPS
      };

      // Begin each layer at 1st color
      led_fade_layer_state_t *state = &s_fade_layer_states[layer];

      esp_err_t status = led_color_fade_init(
         &state->fade,
         s_fade_layers[layer].colors[0],
         s_fade_layers[layer].transition_steps
      );
      if (status != ESP_OK) return status;

      status = led_color_fade_set_target(
         &state->fade,
         s_fade_layers[layer].colors[1]
      );
      if (status != ESP_OK) return status;

      state->target_color_index = 1U;
   }

   return ESP_OK;
}

// Save Fade configuration for one physical layer
esp_err_t led_effects_set_fade_layer(
   size_t layer,
   const led_fade_config_t *config
)
{
   if (layer >= LED_CUBE_SIZE || config == NULL) return ESP_ERR_INVALID_ARG;
   if (config->color_count < 2U || config->color_count > 3U) return ESP_ERR_INVALID_ARG;
   if (config->brightness_percent > 100U) return ESP_ERR_INVALID_ARG;
   if (config->transition_steps == 0U) return ESP_ERR_INVALID_ARG;

   if (s_fade_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_fade_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   // Save only the selected layer configuration
   s_fade_layers[layer] = *config;

   led_fade_layer_state_t *state = &s_fade_layer_states[layer];

   esp_err_t status = led_color_fade_init(
      &state->fade,
      config->colors[0],
      config->transition_steps
   );

   if (status == ESP_OK) status = led_color_fade_set_target(&state->fade, config->colors[1]);
   if (status == ESP_OK) state->target_color_index = 1U;

   xSemaphoreGive(s_fade_mutex);

   return status;
}

// Retrieve the saved Fade configuration for one physical layer
esp_err_t led_effects_get_fade_layer(
   size_t layer,
   led_fade_config_t *config
)
{
   if (layer >= LED_CUBE_SIZE || config == NULL) return ESP_ERR_INVALID_ARG;
   if (s_fade_mutex == NULL) return ESP_ERR_INVALID_STATE;

   if (xSemaphoreTake(s_fade_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   *config = s_fade_layers[layer];
   xSemaphoreGive(s_fade_mutex);

   return ESP_OK;
}

// Restart Fade timing without modifying saved layer configurations
esp_err_t led_effects_fade_start(void)
{
   if (s_fade_mutex == NULL) return ESP_ERR_INVALID_STATE;
   if (xSemaphoreTake(s_fade_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   esp_err_t status = ESP_OK;

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      led_fade_layer_state_t *state = &s_fade_layer_states[layer];
      const led_fade_config_t *config = &s_fade_layers[layer];

      // Restart at color 1, transitioning toward color 2
      status = led_color_fade_init(
         &state->fade,
         config->colors[0],
         config->transition_steps
      );

      if (status != ESP_OK) break;

      status = led_color_fade_set_target(
         &state->fade,
         config->colors[1]
      );

      if (status != ESP_OK) break;

      state->target_color_index = 1U;
   }

   xSemaphoreGive(s_fade_mutex);

   return status;
}

// Save Blink configuration for one physical layer
esp_err_t led_effects_set_blink_layer(
   size_t layer,
   const led_blink_config_t *config
)
{
   if (layer >= LED_CUBE_SIZE || config == NULL) return ESP_ERR_INVALID_ARG;
   if (config->brightness_percent > 100U) return ESP_ERR_INVALID_ARG;
   if (config->interval_ms < 200U || config->interval_ms > 1500U) return ESP_ERR_INVALID_ARG;

   // Acquire exclusive access to shared Blink state
   if (s_blink_mutex == NULL) return ESP_ERR_INVALID_STATE;

   if (xSemaphoreTake(s_blink_mutex, portMAX_DELAY) != pdTRUE)
      return ESP_FAIL;

   // Save layer settings without affecting other layers
   s_blink_layers[layer] = *config;

   // Restart only the updated layer's Blink cycle
   s_blink_layer_states[layer].last_toggle_tick = xTaskGetTickCount();
   s_blink_layer_states[layer].is_on = true;

   // Release shared Blink state after completing the update
   xSemaphoreGive(s_blink_mutex);

   return ESP_OK;
}

// Retrieve the saved configuration for one physical Blink layer
esp_err_t led_effects_get_blink_layer(
   size_t layer,
   led_blink_config_t *config
)
{
   if (layer >= LED_CUBE_SIZE || config == NULL) return ESP_ERR_INVALID_ARG;
   if (s_blink_mutex == NULL) return ESP_ERR_INVALID_STATE;

   // Acquire exclusive access to shared Blink configuration
   if (xSemaphoreTake(s_blink_mutex, portMAX_DELAY) != pdTRUE)
      return ESP_FAIL;

   // Copy the requested layer configuration to the caller
   *config = s_blink_layers[layer];

   // Release the mutex after reading the configuration
   xSemaphoreGive(s_blink_mutex);

   return ESP_OK;
}

esp_err_t led_effects_blink_init(void)
{
   // Create mutex before init of shared Blink state
   if (s_blink_mutex == NULL)
   {
      s_blink_mutex = xSemaphoreCreateMutex();
      if (s_blink_mutex == NULL) return ESP_ERR_NO_MEM;
   }

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      s_blink_layers[layer] = (led_blink_config_t)
      {
         .color = {255U, 0U, 0U},
         .brightness_percent = 100U,
         .interval_ms = 500U
      };

      // Begin each layer in the ON state
      s_blink_layer_states[layer].last_toggle_tick = xTaskGetTickCount();
      s_blink_layer_states[layer].is_on = true;
   }

   return ESP_OK;
}

// Restart Blink timing without changing saved layer configurations
esp_err_t led_effects_blink_start(void)
{
   if (s_blink_mutex == NULL) return ESP_ERR_INVALID_STATE;

   // Acquire exclusive access to share Blink timing state
   if (xSemaphoreTake(s_blink_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;

   TickType_t now = xTaskGetTickCount();

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      s_blink_layer_states[layer].last_toggle_tick = now;
      s_blink_layer_states[layer].is_on = true;
   }

   // Release mutex after resetting layer timeers
   xSemaphoreGive(s_blink_mutex);

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
   if (red > LED_CUBE_MAX_BRIGHTNESS) red = LED_CUBE_MAX_BRIGHTNESS;
   if (green > LED_CUBE_MAX_BRIGHTNESS) green = LED_CUBE_MAX_BRIGHTNESS;
   if (blue > LED_CUBE_MAX_BRIGHTNESS) blue = LED_CUBE_MAX_BRIGHTNESS;

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

// ---------------------------------------
// ----- Public Function Definitions -----
// ---------------------------------------

// Fill the entire cube with a single RGB color
esp_err_t led_effects_set_solid_color(
   uint8_t red,
   uint8_t green,
   uint8_t blue
)
{
   for (size_t x = 0; x < LED_CUBE_SIZE; x++)
   {
      for (size_t y = 0; y < LED_CUBE_SIZE; y++)
      {
         for (size_t z = 0; z < LED_CUBE_SIZE; z++)
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

esp_err_t led_effects_fade_step(void)
{
   led_cube_color_t layer_colors[LED_CUBE_SIZE] = {0};

   if (s_fade_mutex == NULL) return ESP_ERR_INVALID_STATE;

   // Protect configuration and interpolation state during calculations
   if (xSemaphoreTake(s_fade_mutex, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
   
   esp_err_t status = ESP_OK;

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      const led_fade_config_t *config = &s_fade_layers[layer];
      led_fade_layer_state_t *state = &s_fade_layer_states[layer];

      // Advance this layer toward its current target color
      status = led_color_fade_step(&state->fade);
      if (status != ESP_OK) break;

      // Start the next transition when the current color is reached
      if (led_color_fade_is_complete(&state->fade))
      {
         uint8_t next_index = (uint8_t)((state->target_color_index + 1U) % config->color_count);
         status = led_color_fade_set_target(&state->fade, config->colors[next_index]);

         if (status != ESP_OK) break;
         state->target_color_index = next_index;
      }

      // Scale the interpolated RGB color to configured brightness
      layer_colors[layer] = (led_cube_color_t)
      {
         .red = led_blink_scale_channel(
            state->fade.current_color.red,
            config->brightness_percent
         ),
         .green = led_blink_scale_channel(
            state->fade.current_color.green,
            config->brightness_percent
         ),
         .blue = led_blink_scale_channel(
            state->fade.current_color.blue,
            config->brightness_percent
         )
      }; 
   }

   // Release shared state before updating LED framebuffer
   xSemaphoreGive(s_fade_mutex);

   if (status != ESP_OK) return status;

   // Render each layer using independently calculated color
   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      const led_cube_color_t *color = &layer_colors[layer];

      for (size_t y = 0; y < LED_CUBE_SIZE; y++)
      {
         for (size_t x = 0; x < LED_CUBE_SIZE; x++)
         {
            status = led_cube_set_voxel(
               x,
               y,
               layer,
               color->red,
               color->green,
               color->blue
            );

            if (status != ESP_OK) return status;
         }
      }
   }

   // Transmit the completed framebuffer once
   return led_cube_show();
}

/*
   Scale an RGB channel proportionally to the configured brightness.

   Calculation:
      output = (channel * brightness_percent * LED_CUBE_MAX_BRIGHTNESS)
               / (255 * 100)

   - channel (0-255): Requested RGB channel intensity.
   - brightness_percent (0-100): User-selected layer brightness.
   - LED_CUBE_MAX_BRIGHTNESS: Maximum output intensity (currently 75).
   - Dividing by 255 normalizes the RGB channel to a 0-1 ratio.
   - Dividing by 100 converts the brightness percentage to a 0-1 ratio.
   - Adding 12750 (half of 25500) rounds the integer result to nearest.

   Example:
      channel = 128, brightness = 50%, maximum = 75
      output = round((128 * 50 * 75) / 25500) = 19

   Casting to uint32_t before multiplication prevents intermediate overflow.
*/
static uint8_t led_blink_scale_channel(
   uint8_t channel,
   uint8_t brightness_percent
)
{
   uint32_t scaled = (uint32_t)channel * (uint32_t)brightness_percent * LED_CUBE_MAX_BRIGHTNESS;

   return (uint8_t)((scaled + 12750U) / 25500U);
}

// Render all eight Blink layers with independent timing and color settings
esp_err_t led_effects_blink_step(void)
{
   led_cube_color_t layer_colors[LED_CUBE_SIZE] = {0};

   // Verify synchronization is initialized
   if (s_blink_mutex == NULL) return ESP_ERR_INVALID_STATE;

   // Acquire exclusive access to Blink configuration and timing state
   if (xSemaphoreTake(s_blink_mutex, portMAX_DELAY) != pdTRUE)
      return ESP_FAIL;

   TickType_t now = xTaskGetTickCount();

   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      const led_blink_config_t *config = &s_blink_layers[layer];
      led_blink_layer_state_t *state = &s_blink_layer_states[layer];

      // Convert configured ON/OFF duration into FreeRTOS ticks
      TickType_t interval_ticks = pdMS_TO_TICKS(config->interval_ms);

      // Toggle only this layer when its interval expires
      if ((TickType_t)(now - state->last_toggle_tick) >= interval_ticks)
      {
         state->is_on = !state->is_on;
         state->last_toggle_tick = now;
      }

      // Calculate this layer's RGB output while holding the mutex
      layer_colors[layer] = (led_cube_color_t)
      {
         .red = state->is_on
            ? led_blink_scale_channel(config->color.red, config->brightness_percent)
            : 0U,

         .green = state->is_on
            ? led_blink_scale_channel(config->color.green, config->brightness_percent)
            : 0U,

         .blue = state->is_on
            ? led_blink_scale_channel(config->color.blue, config->brightness_percent)
            : 0U
      };
   }

   // Configuration and timing state are no longer accessed after this point
   xSemaphoreGive(s_blink_mutex);

   // Render the locally calculated colors into the framebuffer
   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      const led_cube_color_t *color = &layer_colors[layer];

      for (size_t y = 0; y < LED_CUBE_SIZE; y++)
      {
         for (size_t x = 0; x < LED_CUBE_SIZE; x++)
         {
            esp_err_t status = led_cube_set_voxel(
               x,
               y,
               layer,
               color->red,
               color->green,
               color->blue
            );

            if (status != ESP_OK) return status;
         }
      }
   }

   // Transmit the completed frame once
   return led_cube_show();
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
