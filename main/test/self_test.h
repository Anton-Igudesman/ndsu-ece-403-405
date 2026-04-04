#ifndef SELF_TEST_H
#define SELF_TEST_H

#include <driver/i2s_std.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t self_test_run_all(void);
esp_err_t self_test_start_mic_monitor(i2s_chan_handle_t rx_chan);

esp_err_t self_test_log_eq_columns_text(
   const float *bands_norm,
   size_t num_bands,
   uint8_t matrix_height);

#endif