#ifndef APP_LOG_H
#define APP_LOG_H

#include "esp_err.h"

void app_log_error(const char *tag, const char *name, esp_err_t status);

#endif