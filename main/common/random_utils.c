#include "random_utils.h"

#include <stdint.h>

#include "esp_random.h"

float random_float_range(
   float min_value,
   float max_value
)
{
   // Normalize the 32-bit random value into the range 0.0 - 1.0
   float normalized =
      (float)esp_random() / (float)UINT32_MAX;

   // Scale the normalized value into the requested range
   return min_value +
      (normalized * (max_value - min_value));
}

uint32_t random_uint32_range(
   uint32_t min_value,
   uint32_t max_value
)
{
   // Return the lower bound if the requested range has no width
   if (min_value >= max_value) return min_value;

   uint32_t range = max_value - min_value + 1U;

   return min_value + (esp_random() % range);
}

// Generate a random 8-bit value within the requested inclusive range
uint8_t random_uint8_range(
   uint8_t min_value,
   uint8_t max_value
)
{
   if (min_value >= max_value) return min_value;

   return (uint8_t)random_uint32_range(
      (uint32_t)min_value,
      (uint32_t)max_value
   );
}