#ifndef AUDIO_BUFFER_H
#define AUDIO_BUFFER_H 

// Hz per bin = sample_rate/hz / FFT size => 43.875 Hz/bin 
#define AUDIO_FRAME_SIZE 1024 // For accurate visual-EQ bin behavior

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t audio_buffer_init(void);
esp_err_t audio_buffer_push_sample(int16_t sample);
esp_err_t audio_buffer_try_get_frame(const int16_t **frame);

#endif