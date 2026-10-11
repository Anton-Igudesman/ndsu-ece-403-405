#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_err.h"

esp_err_t wifi_ap_init(void);
esp_err_t wifi_ap_start(void);
esp_err_t wifi_ap_stop(void);

#endif