#include "led_controller.h"

#include <stdbool.h>

static led_mode_t s_active_mode;
static bool s_initialized = false;