#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t ble_telemetry_init(void);
esp_err_t ble_telemetry_start(void);

esp_err_t ble_telemetry_update_snapshot(
   uint8_t peak_bin,
   float peak_hz,
   const float *bands_norm,
   size_t num_bands
);

#endif