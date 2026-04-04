#include <stdint.h>
#include <stdbool.h>
#include <driver/i2s_std.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_err.h"
#include "esp_log.h"
#include "led_strip.h"

// Custom headers
#include "led/led_protocol.h"
#include "led/led_effects.h"
#include "led/led_matrix_renderer.h"
#include "ble/ble_telemetry.h"
#include "dsp/audio_buffer.h"
#include "dsp/fft_engine.h"
#include "dsp/audio_dsp_pipeline.h"
#include "test/self_test.h"
#include "common/app_log.h"

_Static_assert(AUDIO_FRAME_SIZE == 1024, "Unexpected AUDIO_FRAME_SIZE");
_Static_assert(AUDIO_FRAME_SIZE == FFT_FRAME_SIZE,
"AUDIO_FRAME_SIZE must equal FFT_FRAME_SIZE");

#define DSP_DEBUG_LOGS 1
#define MIC_DEBUG_MONITOR 0 // 1 enables debug mode for mic input
#define LED_GPIO GPIO_NUM_4
#define LED_AUDIO_QUEUE_LEN 1U // Queue consists of 1 processed audio frame
#define RGB_LED_INDEX 0
#define LED_DEFAULT_BLINK_MS 500
#define LED_CMD_QUEUE_LEN 8
#define LED_ON 1
#define LED_OFF 0
#define MATRIX_WIDTH 8
#define MATRIX_HEIGHT 8
#define MATRIX_PIXELS (MATRIX_WIDTH * MATRIX_HEIGHT)

static const char *LOG_TAG = "rtos_ble_led";
static led_strip_handle_t s_led_strip = NULL; 
static i2s_chan_handle_t s_i2s_rx_chan = NULL;
static uint32_t s_fft_frame_counter = 0; // Frame counter for rate-limited logs

typedef struct
{
   float bands_norm[SPECTRUM_NUM_BANDS];
} led_audio_frame_t;

static QueueHandle_t s_led_audio_queue = NULL;

// ----------------------------------------------------------- 
// -------------------- Module Scope -------------------------
// -----------------------------------------------------------

static void log_effect_status(const char *op, esp_err_t status)
{
   if (status == ESP_OK) ESP_LOGI(LOG_TAG, "%s: ESP_OK", op);
   else ESP_LOGE(LOG_TAG, "%s: %s", op, esp_err_to_name(status));
}

// Reads I2S chunk, converts samples, pushes into audio buffer
static esp_err_t mic_read_and_push_samples(void)
{
   int32_t raw_samples[256];
   size_t bytes_read = 0;

   esp_err_t status = i2s_channel_read(
      s_i2s_rx_chan,
      raw_samples,
      sizeof(raw_samples),
      &bytes_read,
      pdMS_TO_TICKS(1000)
   );

   if (status != ESP_OK) return status; 

   // How many 32 bit samples do we have so far
   size_t sample_count = bytes_read / sizeof(int32_t);

   for (size_t i = 0; i < sample_count; i++)
   {
      int32_t sample32 = raw_samples[i] >> 8; // align 24-bit payload to bits [23:0]

      // If 24-bit sign bit is 1
      if ((sample32 & 0x00800000) != 0) sample32 |= 0xFF000000;

      // Downscale to int16_t for current audio buffer/FFT path
      int16_t sample16 = (int16_t)(sample32 >> 8); // Proper resolution reduction
      audio_buffer_push_sample(sample16);
   }

   return ESP_OK;
}

// Initialize microphone
static esp_err_t mic_i2s_init(void)
{
   i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(
      I2S_NUM_AUTO,
      I2S_ROLE_MASTER
   );

   esp_err_t status = i2s_new_channel(&channel_config, NULL, &s_i2s_rx_chan);
   if (status != ESP_OK) return status;

   // Hardware config and pinout
   i2s_std_config_t std_config = 
   {
      // Max freq = sample_rate / 2
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
         I2S_DATA_BIT_WIDTH_32BIT,
         I2S_SLOT_MODE_MONO
      ),
      .gpio_cfg = 
      {
         .mclk = I2S_GPIO_UNUSED,
         .bclk = GPIO_NUM_16,
         .ws = GPIO_NUM_17,
         .dout = I2S_GPIO_UNUSED,
         .din = GPIO_NUM_18,
         .invert_flags = 
         {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv = false,
         },
      },
   };

   // L/R pint tied to GND => LEFT channel mono mode
   std_config.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
   status = i2s_channel_init_std_mode(s_i2s_rx_chan, &std_config);
   if (status != ESP_OK) return status;
   
   return i2s_channel_enable(s_i2s_rx_chan);
}

// Own mode dispatch for LED patterns
static esp_err_t led_process_mode_step(
   led_mode_t mode,
   led_audio_frame_t *latest_audio,
   bool *led_level
)
{
   esp_err_t status = ESP_OK;

   if (latest_audio == NULL || led_level == NULL) return ESP_ERR_INVALID_ARG;

   if (mode == LED_MODE_BLINK)
   {
      *led_level = !(*led_level);

      status = led_effects_set_color_level(LED_COLOR_RED, *led_level);
      log_effect_status("led_effects_set_color_level", status);

      ESP_LOGI(LOG_TAG, "led state is %d", *led_level);
   }

   else if (mode == LED_MODE_FADE) status = led_effects_fade_step();
   else if (mode == LED_MODE_BREATHE)
   {
      status = led_effects_breathe_step(LED_COLOR_RED, 55, 255);
      log_effect_status("led_effects_breathe_step", status);
   }

   else if (mode == LED_MODE_AUDIO_EQ)
   {
      status = led_matrix_render_eq(latest_audio->bands_norm);
      if (status != ESP_OK) log_effect_status("led_matrix_render_eq", status);
   }

   else return ESP_ERR_INVALID_ARG;

   return status;
}

static void led_task(void *arg)
{
   (void)arg;

   bool led_level = false;
   uint32_t blink_period_ms = LED_DEFAULT_BLINK_MS;
   esp_err_t status = ESP_OK;

   led_mode_t mode = LED_MODE_AUDIO_EQ; // Choose between color fader and blinky
   led_audio_frame_t latest_audio = {0};

   // Based on which LED pattern is set in mode variable above
   while (true)
   {
      // Non-blocking read of latest audio frame - keep last value if none available
      (void)xQueueReceive(s_led_audio_queue, &latest_audio, 0);

      // Abstract away LED pattern algorithms
      status = led_process_mode_step(
         mode,
         &latest_audio,
         &led_level);

      if (status != ESP_OK) log_effect_status("led_process_mode_step", status);

      if (mode == LED_MODE_BLINK) vTaskDelay(pdMS_TO_TICKS(blink_period_ms));
      else vTaskDelay(pdMS_TO_TICKS(10));
   }
}

static void audio_publish_bands_to_led_queue(const float *bands_norm)
{
   if (bands_norm == NULL) return;

   led_audio_frame_t frame_msg = {0};

   // Copy current normalized bands into queue payload snapshot
   for (size_t band_index = 0; band_index < SPECTRUM_NUM_BANDS; band_index++)
   {
      frame_msg.bands_norm[band_index] = bands_norm[band_index];
   }

   // Overwrite stale frame if queue already has one
   if (xQueueOverwrite(s_led_audio_queue, &frame_msg) != pdPASS)
   {
      ESP_LOGW(LOG_TAG, "xQueueOverwrite failed");
   }
}

static bool audio_should_continue_no_frame_path(
   esp_err_t process_status,
   bool frame_processed,
   uint32_t *no_frame_count
)
{
   if (no_frame_count == NULL) return false;

   if (process_status == ESP_ERR_NOT_FOUND
      || (process_status == ESP_OK && !frame_processed)
   )
   {
      (*no_frame_count)++;

      if (((*no_frame_count) % 200U) == 0U) 
      {
         ESP_LOGW(LOG_TAG, "no full frame yet (count=%lu)", (unsigned long)(*no_frame_count));
      }

      vTaskDelay(pdMS_TO_TICKS(2));
      return true;
   }

   return false;
}

static void audio_log_dsp_debug(const float *bands_norm)
{
   if (bands_norm == NULL) return;

   ESP_LOGI(LOG_TAG, "bands_norm: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      bands_norm[0], bands_norm[1], bands_norm[2], bands_norm[3],
      bands_norm[4], bands_norm[5], bands_norm[6], bands_norm[7]);

   float peak_hz = 0.0f;
   size_t peak_bin = 0;
   float peak_mag = 0.0f;

   esp_err_t status = audio_dsp_get_peak_frequency(&peak_hz, &peak_bin, &peak_mag);
   if (status == ESP_OK)
   {
      ESP_LOGI(LOG_TAG, "fft_peak: bin=%u hz=%.2f mag=%.2f",
         (unsigned)peak_bin,
         peak_hz,
         peak_mag);
   }
   
   else app_log_error(LOG_TAG, "audio_dsp_get_peak_frequency", status);

   float band_peak_hz[SPECTRUM_NUM_BANDS] = {0};
   float band_peak_mag[SPECTRUM_NUM_BANDS] = {0};

   status = audio_dsp_get_band_peak_frequency(
      band_peak_hz,
      band_peak_mag,
      SPECTRUM_NUM_BANDS);

   if (status == ESP_OK)
   {
      ESP_LOGI(LOG_TAG,
         "band_peaks_hz: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
         band_peak_hz[0], band_peak_hz[1], band_peak_hz[2], band_peak_hz[3],
         band_peak_hz[4], band_peak_hz[5], band_peak_hz[6], band_peak_hz[7]);

      ESP_LOGI(LOG_TAG,
         "band_peaks_mag: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
         band_peak_mag[0], band_peak_mag[1], band_peak_mag[2], band_peak_mag[3],
         band_peak_mag[4], band_peak_mag[5], band_peak_mag[6], band_peak_mag[7]);
   }
   else app_log_error(LOG_TAG, "audio_dsp_get_band_peak_frequency", status);
   

   status = self_test_log_eq_columns_text(bands_norm, SPECTRUM_NUM_BANDS, MATRIX_HEIGHT);
   if (status != ESP_OK) app_log_error(LOG_TAG, "self_test_log_eq_columns_text", status);
}

// Live mic -> audio buffer -> FFT -> bands
static void audio_fft_live_task(void *arg)
{
   (void)arg;
   audio_buffer_init();

   esp_err_t status = fft_engine_init();
   app_log_error(LOG_TAG, "fft_engine_init", status);
   if (status != ESP_OK)
   {
      vTaskDelete(NULL);
      return;
   }

   static uint32_t s_no_frame_count = 0;
   while (true)
   {
      status = mic_read_and_push_samples();
      if (status != ESP_OK)
      {
         app_log_error(LOG_TAG, "mic_read_and_push_samples", status);
         vTaskDelay(2);
         continue;
      }

      float bands_norm[SPECTRUM_NUM_BANDS] = {0};
      bool frame_processed = false;

      status = audio_dsp_process_ready_frame(bands_norm, &frame_processed);
      
      if (audio_should_continue_no_frame_path(
         status, 
         frame_processed,
         &s_no_frame_count)) continue;

      if (status != ESP_OK)
      {
         app_log_error(LOG_TAG, "process_ready_audio_frame", status);
         vTaskDelay(pdMS_TO_TICKS(2));
         continue;
      }

      s_no_frame_count = 0;
      s_fft_frame_counter++;
      audio_publish_bands_to_led_queue(bands_norm);

      // Throttle DSP/log path: keep 1 in 8 processed frames for display/log
      if ((s_fft_frame_counter % 8U) != 0U)
      {
         vTaskDelay(pdMS_TO_TICKS(5));
         continue;
      }

      #if DSP_DEBUG_LOGS 
      audio_log_dsp_debug(bands_norm);
      #else 
      if ((s_fft_frame_counter % 64U) == 0U) ESP_LOGI(LOG_TAG, "audio frame processed");
      #endif

      // Always yield so IDLE task can run and feed watchdog
      vTaskDelay(5);
   }
}

void app_main(void)
{

   // LED strip configuration
   led_strip_config_t strip_config = {
      .strip_gpio_num = LED_GPIO,
      .max_leds = MATRIX_PIXELS,
      .led_model = LED_MODEL_WS2812,
      .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
      .flags = {
         .invert_out = false,
      },
   };

   // Transmit settings
   led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = 10 * 1000 * 1000,   
      .mem_block_symbols = 64,
      .flags = {
         .with_dma = false,
      },
   };

   esp_err_t status = led_strip_new_rmt_device(
      &strip_config, 
      &rmt_config, 
      &s_led_strip);

   ESP_LOGI(LOG_TAG, "new_rmt_device: %d (%s)", status, esp_err_to_name(status));
    
   if (status != ESP_OK) {
      ESP_LOGE(LOG_TAG, "led_strip_new_rmt_device failed");
      return;
   }

   status = led_effects_init(s_led_strip, RGB_LED_INDEX);
   if (status != ESP_OK)
   {
      ESP_LOGE(LOG_TAG, "led_effects_init failed: %s", esp_err_to_name(status));
      return;
   }

   status = led_matrix_renderer_init(s_led_strip);
   if (status != ESP_OK)
   {
      ESP_LOGE(LOG_TAG, "led_matrix_renderer_init failed: %s", esp_err_to_name(status));
      return;
   }

   status = mic_i2s_init();
   app_log_error(LOG_TAG, "mic_i2s_init", status);
   if (status != ESP_OK) return;

   // Mailbox queue for latest DSP-normalized band frame -> LED task
   s_led_audio_queue = xQueueCreate(LED_AUDIO_QUEUE_LEN, sizeof(led_audio_frame_t));
   if (s_led_audio_queue == NULL)
   {
      ESP_LOGE(LOG_TAG, "Failed to create s_led_audio_queue");
      return;
   }

   // Initialize BLE beacon
   status = ble_telemetry_init();

   if (status != ESP_OK) ESP_LOGE(LOG_TAG, "ble_telemetry_init failed: %s", esp_err_to_name(status));
   else
   {
      status = ble_telemetry_start();
      if (status != ESP_OK) ESP_LOGE(LOG_TAG, "ble_telemetry_start failed: %s", esp_err_to_name(status));
   }

   #if MIC_DEBUG_MONITOR
   status = self_test_start_mic_monitor(s_i2s_rx_chan);
   if (status != ESP_OK)
   {
      ESP_LOGE(LOG_TAG, "self_test_start_mic_monitor failed: %s", esp_err_to_name(status));
      return;
   }

   return;
   #endif

   if (xTaskCreatePinnedToCore(audio_fft_live_task, "audio_fft_live_task", 6144, NULL, 4, NULL, 1) != pdPASS)
   {
      ESP_LOGE(LOG_TAG, "Failed to create audio_fft_live_task");
      return;
   }

   if (xTaskCreate(led_task, "led_task", 4096, NULL, 5, NULL) != pdPASS)
   {
      ESP_LOGE(LOG_TAG, "Failed to create led_task");
      return;
   }  
}


