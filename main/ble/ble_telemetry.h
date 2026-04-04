#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include "esp_err.h"

esp_err_t ble_telemetry_init(void);
esp_err_t ble_telemetry_start(void);

#endif