#pragma once

#include "esp_err.h"
#include "cJSON.h"

// Validate and execute a terminal command received from the web interface
esp_err_t web_terminal_execute(const cJSON *command);

