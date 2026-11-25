#pragma once

#include "esp_err.h"

#include <stdbool.h>

esp_err_t led_effect_text_init(bool start);
esp_err_t led_effect_text_step(void);