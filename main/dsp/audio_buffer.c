#include "audio_buffer.h"
#include "esp_err.h"

static int16_t s_frame[AUDIO_FRAME_SIZE]; // Frame will hold individual audio samples for processing
static size_t s_write_index = 0;
static bool s_frame_ready = false;

esp_err_t audio_buffer_init(void)
{
   // Zeros to all s_frame index for init
   for (size_t i = 0; i < AUDIO_FRAME_SIZE; i++) s_frame[i] = 0;
   
   s_write_index = 0;
   s_frame_ready = false;
   return ESP_OK;
}

// Push single audio sample into frame
esp_err_t audio_buffer_push_sample(int16_t sample)
{
   // Frame is ready for processing
   if (s_frame_ready) return ESP_OK;

   s_frame[s_write_index++] = sample;

   if (s_write_index >= AUDIO_FRAME_SIZE)
   {
      s_write_index = 0;
      s_frame_ready = true;
   }

   return ESP_OK;
}

esp_err_t audio_buffer_try_get_frame(const int16_t **frame)
{
   // Frame not ready
   if (frame == NULL) return ESP_ERR_INVALID_ARG; 
   if (!s_frame_ready) return ESP_ERR_NOT_FOUND;;
   
   *frame = s_frame;
   s_frame_ready = false;
   return ESP_OK;
}
