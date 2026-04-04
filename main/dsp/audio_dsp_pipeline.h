#ifndef AUDIO_DSP_PIPELINE_H
#define AUDIO_DSP_PIPELINE_H

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "spectrum_map.h"

esp_err_t audio_dsp_process_ready_frame(
   float *bands_norm_out, 
   bool *frame_processed);

esp_err_t audio_dsp_get_peak_frequency(
   float *peak_hz_out,
   size_t *peak_bin_out,
   float *peak_mag_out);

esp_err_t audio_dsp_get_band_peak_frequency(
   float *band_peak_hz_out,
   float *band_peak_mag_out,
   size_t num_bands);

esp_err_t audio_dsp_get_last_band_peaks(
   float *band_peak_hz_out,
   float *band_peak_mag_out,
   size_t num_bands);

#endif