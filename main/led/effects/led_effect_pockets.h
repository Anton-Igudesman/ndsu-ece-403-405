#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "led_effects.h"

// Defines a spherical color pocket within the Logical cube
typedef struct
{
   // Current center position of the pocket
   float x;
   float y;
   float z;

   // Movement applied to the pocket center on each update
   float velocity_x;
   float velocity_y;
   float velocity_z;

   // Current pocket size and amount changes on each update
   float radius;
   float radius_step;

   // Limits used to keep pocket from becoming too small/large
   float min_radius;
   float max_radius;
} led_effect_pocket_t;

// Initialize the color-pocket effect state
esp_err_t led_effect_pockets_init(void);

// Advance and render one frame of the color-pocket effect
esp_err_t led_effect_pockets_step(void);