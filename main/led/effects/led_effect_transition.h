#pragma once

#include <stdbool.h>

#include "esp_err.h"

esp_err_t led_effect_transition_init(void);

esp_err_t led_effect_transition_step(
   bool *complete
);