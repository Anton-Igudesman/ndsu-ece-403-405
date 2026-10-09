#include "led_effect_pockets.h"
#include "led_effects.h"
#include "led_cube.h"
#include "led_color_fade.h"
#include "random_utils.h"

#include <stdbool.h>
#include <stddef.h>

// ----------- Module Scope Definitions ------------

// Pocket geometry and movement state
static led_effect_pocket_t s_pocket;

// Independent color transitions for pocket and surrounding background
static led_color_fade_t s_pocket_fade;
static led_color_fade_t s_background_fade;

// Tracket whether pocket effect is initialized
static bool s_initialized = false;

// -----------------------------------
// ---- Small helpers ----------------
// -----------------------------------
// Generate one random RGB channel value within the effect brightness limit
static uint8_t led_effect_pockets_random_channel(void)
{
   return random_uint8_range(
      0U,
      LED_EFFECT_FILL_MAX_BRIGHTNESS
   );
}

// Generate a random RGB color within the allowed effect brightness range
static led_cube_color_t led_effect_pockets_random_color(void)
{
   led_cube_color_t color =
   {
      .red = led_effect_pockets_random_channel(),
      .green = led_effect_pockets_random_channel(),
      .blue = led_effect_pockets_random_channel()
   };

   return color;
}

// Position a pocket outside the cube so it can move naturally into view
static void led_effect_pockets_set_entry(
   led_effect_pocket_t *pocket,
   led_effect_direction_t direction,
   float travel_speed
)
{
   if (pocket == NULL) return;

   float cube_max = (float)(LED_CUBE_SIZE - 1U);

   switch (direction)
   {
      case LED_EFFECT_DIRECTION_X_POS:
         // Enter from the negative X side and move toward increasing X
         pocket->x = -pocket->radius;
         pocket->velocity_x = travel_speed;
         break;

      case LED_EFFECT_DIRECTION_X_NEG:
         // Enter from the positive X side and move toward decreasing X
         pocket->x = cube_max + pocket->radius;
         pocket->velocity_x = -travel_speed;
         break;

      case LED_EFFECT_DIRECTION_Y_POS:
         // Enter from the negative Y side and move toward increasing Y
         pocket->y = -pocket->radius;
         pocket->velocity_y = travel_speed;
         break;

      case LED_EFFECT_DIRECTION_Y_NEG:
         // Enter from the positive Y side and move toward decreasing Y
         pocket->y = cube_max + pocket->radius;
         pocket->velocity_y = -travel_speed;
         break;

      case LED_EFFECT_DIRECTION_Z_POS:
         // Enter below the cube and move toward increasing Z
         pocket->z = -pocket->radius;
         pocket->velocity_z = travel_speed;
         break;

      case LED_EFFECT_DIRECTION_Z_NEG:
         // Enter above the cube and move toward decreasing Z
         pocket->z = cube_max + pocket->radius;
         pocket->velocity_z = -travel_speed;
         break;

      default:
         break;
   }
}

// Reconfigure a pocket after it has completely moved outside the cube
static void led_effect_pockets_respawn(led_effect_pocket_t *pocket)
{
   if (pocket == NULL) return;

   float cube_max = (float)(LED_CUBE_SIZE - 1U);

   // Select one of the six cube entry directions
   led_effect_direction_t direction =
      (led_effect_direction_t)random_uint32_range(
         LED_EFFECT_DIRECTION_X_POS,
         LED_EFFECT_DIRECTION_Z_NEG
      );

   // Select a new starting radius within the pocket's configured limits
   pocket->radius = random_float_range(
      pocket->min_radius,
      pocket->max_radius
   );

   // Randomize the pocket position within the other two cube axes
   pocket->x = random_float_range(0.0f, cube_max);
   pocket->y = random_float_range(0.0f, cube_max);
   pocket->z = random_float_range(0.0f, cube_max);

   // Give the pocket a small random drift on all three axes
   pocket->velocity_x = random_float_range(-0.04f, 0.04f);
   pocket->velocity_y = random_float_range(-0.04f, 0.04f);
   pocket->velocity_z = random_float_range(-0.04f, 0.04f);

   // Randomize how quickly the pocket expands or contracts
   pocket->radius_step = random_float_range(0.01f, 0.03f);

   // Randomly begin this pocket in either expansion or contraction
   if (random_uint32_range(0U, 1U) == 0U) pocket->radius_step = -pocket->radius_step;

   // Select a guaranteed non-zero speed for movement through the entry axis
   float travel_speed = random_float_range(
      0.02f,
      0.06f
   );

   // Position the pocket outside the selected entry face and force
   // the primary velocity component to point into the cube
   led_effect_pockets_set_entry(
      pocket,
      direction,
      travel_speed
   );
}

// Determine whether the entire pocket has moved outside the logical cube
static bool led_effect_pockets_is_outside(
   const led_effect_pocket_t *pocket
)
{
   if (pocket == NULL) return true;

   // Include the pocket radius so it is not considered outside
   // until its entire spherical region has cleared the cube
   if ((pocket->x + pocket->radius) < 0.0f) return true;
   if ((pocket->x - pocket->radius) > (float)(LED_CUBE_SIZE - 1U)) return true;

   if ((pocket->y + pocket->radius) < 0.0f) return true;
   if ((pocket->y - pocket->radius) > (float)(LED_CUBE_SIZE - 1U)) return true;

   if ((pocket->z + pocket->radius) < 0.0f) return true;
   if ((pocket->z - pocket->radius) > (float)(LED_CUBE_SIZE - 1U)) return true;

   return false;
}

// Update pocket position and size for one animation step
static void led_effect_pockets_update(led_effect_pocket_t *pocket)
{
   if (pocket == NULL) return;

   // Move the pocket center according to its current velocity
   pocket->x += pocket->velocity_x;
   pocket->y += pocket->velocity_y;
   pocket->z += pocket->velocity_z;

   // Expand or contract the pocket
   pocket->radius += pocket->radius_step;

   // Reverse radius direction when the configured limits are reached
   if (pocket->radius >= pocket->max_radius)
   {
      pocket->radius = pocket->max_radius;
      pocket->radius_step = -pocket->radius_step;
   }
   else if (pocket->radius <= pocket->min_radius)
   {
      pocket->radius = pocket->min_radius;
      pocket->radius_step = -pocket->radius_step;
   }

   // Generate a new pocket after the current pocket fully exits the cube
   if (led_effect_pockets_is_outside(pocket)) led_effect_pockets_respawn(pocket);
}

// Determine whether a logical voxel falls inside a spherical color pocket
static bool led_effect_pockets_voxel_is_inside(
   size_t x,
   size_t y,
   size_t z,
   const led_effect_pocket_t *pocket
)
{
   if (pocket == NULL) return false;

   //Calculate the voxel's offset from the pocket center on each axis
   float dx = (float)x - pocket->x;
   float dy = (float)y - pocket->y;
   float dz = (float)z - pocket->z;

   // Compare squared distance against squared radius to avoid sqrt()
   float distance_squared = 
      (dx * dx) +
      (dy * dy) +
      (dz * dz);

   float radius_squared = pocket->radius * pocket->radius;

   return distance_squared <= radius_squared;
}

// Render the current pocket and background colors across the logical cube
static esp_err_t led_effect_pockets_render(void)
{
   for (size_t z = 0; z < LED_CUBE_SIZE; z++)
   {
      for (size_t y = 0; y < LED_CUBE_SIZE; y++)
      {
         for (size_t x = 0; x < LED_CUBE_SIZE; x++)
         {
            // Select the pocket or background fade based on voxel position
            led_cube_color_t color =
               led_effect_pockets_voxel_is_inside(
                  x,
                  y,
                  z,
                  &s_pocket
               )
               ? s_pocket_fade.current_color
               : s_background_fade.current_color;

            // Stage the selected color for this logical voxel
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

   // Display the completed 512-voxel frame at once
   return led_cube_show();
}

// Initialize the moving color-pocket effect
esp_err_t led_effect_pockets_init(void)
{
   // Reset all pocket state before applying the initial configuration
   s_pocket = (led_effect_pocket_t){0};

   // Define the allowed pocket size range in logical cube coordinates
   s_pocket.min_radius = 1.0f;
   s_pocket.max_radius = 3.5f;

   // Generate the initial randomized pocket outside the cube
   led_effect_pockets_respawn(&s_pocket);

   led_cube_color_t pocket_start = 
   {
      .red = 0,
      .green = 0,
      .blue = LED_EFFECT_FILL_MAX_BRIGHTNESS
   };

   led_cube_color_t background_start = 
   {
      .red = LED_EFFECT_FILL_MAX_BRIGHTNESS,
      .green = 0,
      .blue = 0
   };

   esp_err_t status = led_color_fade_init(
      &s_pocket_fade,
      pocket_start,
      100U
   );
   if (status != ESP_OK) return status;

   status = led_color_fade_init(
      &s_background_fade,
      background_start,
      150U
   );
   if (status != ESP_OK) return status;

   // Select the first random target colors for both independent fades
   status = led_color_fade_set_target(
      &s_pocket_fade,
      led_effect_pockets_random_color()
   );
   if (status != ESP_OK) return status;

   status = led_color_fade_set_target(
      &s_background_fade,
      led_effect_pockets_random_color()
   );
   if (status != ESP_OK) return status;

   s_initialized = true;
   return ESP_OK;
}

// Advance the color-pocket effect by one animation frame
esp_err_t led_effect_pockets_step(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;

   // Advance pocket position, radius, and respawn when fully outside
   led_effect_pockets_update(&s_pocket);

   // Advance the pocket's independent color transition
   esp_err_t status = led_color_fade_step(&s_pocket_fade);
   if (status != ESP_OK) return status;

   // Select a new random pocket color after the current fade completes
   if (led_color_fade_is_complete(&s_pocket_fade))
   {
      status = led_color_fade_set_target(
         &s_pocket_fade,
         led_effect_pockets_random_color()
      );
      if (status != ESP_OK) return status;
   }

   // Advance the background color transition independently
   status = led_color_fade_step(&s_background_fade);
   if (status != ESP_OK) return status;

   // Select a new random background color after the current fade completes
   if (led_color_fade_is_complete(&s_background_fade))
   {
      status = led_color_fade_set_target(
         &s_background_fade,
         led_effect_pockets_random_color()
      );
      if (status != ESP_OK) return status;
   }

   return led_effect_pockets_render();
}
