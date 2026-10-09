#pragma once

#include <stdint.h>

// Generate a random floating-point value within the requested range
float random_float_range(
   float min_value,
   float max_value
);

// Generate a random unsigned integer within the requested inclusive range
uint32_t random_uint32_range(
   uint32_t min_value,
   uint32_t max_value
);

// Generate a random 8-bit value within the requested inclusive range
uint8_t random_uint8_range(
   uint8_t min_value,
   uint8_t max_value
);