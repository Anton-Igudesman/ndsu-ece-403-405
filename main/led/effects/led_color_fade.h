#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#include "led_cube.h"

/*
   State for a smooth transition between two arbitrary RGB colors.
*/
typedef struct
{
   // Fixed color where the current transition began
   led_cube_color_t start_color;

   // Color being approached during the current transition
   led_cube_color_t target_color;

   // RGB value produced for the current animation frame
   led_cube_color_t current_color;

   // Total number of animation updates used to complete the transition
   uint32_t transition_steps;

   // Current position within the active transition
   uint32_t current_step;
} led_color_fade_t;

// Initialize a color fade with a starting color and transition length
esp_err_t led_color_fade_init(
   led_color_fade_t *fade,
   led_cube_color_t start_color,
   uint32_t transition_steps
);

// Set the next color that the fade should transition toward
esp_err_t led_color_fade_set_target(
   led_color_fade_t *fade,
   led_cube_color_t target_color
);

// Advance the fade by one animation step
esp_err_t led_color_fade_step(
   led_color_fade_t *fade
);

// Return true when the active color transition has reached its target
bool led_color_fade_is_complete(
   const led_color_fade_t *fade
);