#pragma once

#include "esp_err.h"
#include "cJSON.h"
#include "led_protocol.h"

// Validate and execute a terminal command received from the web interface
// Return active LED mode for get_mode requests
esp_err_t web_terminal_execute(const cJSON *command, led_mode_t *active_mode);

