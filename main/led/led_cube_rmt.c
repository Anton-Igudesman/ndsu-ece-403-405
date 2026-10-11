#include "led_cube_rmt.h"

#include <stdbool.h>
#include <string.h>

#include "driver/rmt_tx.h"
#include "esp_log.h"

// -------------------------------
// --- RMT Driver Configuration --
// -------------------------------

#define LED_CUBE_RMT_RESOLUTION_HZ 10000000U
#define LED_CUBE_RMT_MEM_SYMBOLS 48U

/*
   WS2812 bit timing converted into RMT clock ticks.

   RMT resolution is 10 MHz:
      1 tick = 1 / 10 MHz = 100 ns

   WS2812 data bits are encoded by varying the HIGH and LOW pulse widths:
      Bit 0: ~400 ns HIGH, ~900 ns LOW
      Bit 1: ~800 ns HIGH, ~500 ns LOW
*/
#define WS2812_T0H_TICKS 4U   // 4 × 100 ns = 400 ns HIGH
#define WS2812_T0L_TICKS 9U   // 9 × 100 ns = 900 ns LOW

#define WS2812_T1H_TICKS 8U   // 8 × 100 ns = 800 ns HIGH
#define WS2812_T1L_TICKS 5U   // 5 × 100 ns = 500 ns LOW

// -------------------------------
// --- Module Scope Definitions --
// -------------------------------

static const char *LOG_TAG = "led_cube_rmt";

// Four reusable RMT TX channels drive the eight physical cube layers
static rmt_channel_handle_t s_tx_channels[LED_CUBE_RMT_CHANNEL_COUNT] = {0};

// Shared encoder converts layer pixel data into WS2812 RMT symbols
static rmt_encoder_handle_t s_ws2812_encoder = NULL;

// Store the physical data GPIO assigned to each cube layer
static int s_layer_gpios[LED_CUBE_SIZE] = {0};

// Store GRB-formatted bytes prepared for transmission to each cube layer
static uint8_t s_tx_buffer
   [LED_CUBE_SIZE]
   [LED_CUBE_LAYER_PIXELS * 3U] = {0};

static bool s_initialized = false;

// ----------------------------------------
// ----- Private Function Definitions -----
// ----------------------------------------

// Create one reusable RMT TX channel initially assigned to a layer GPIO
static esp_err_t create_tx_channel(
   size_t channel_index,
   int gpio_num
)
{
   if (channel_index >= LED_CUBE_RMT_CHANNEL_COUNT)
      return ESP_ERR_INVALID_ARG;

   rmt_tx_channel_config_t channel_config =
   {
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .gpio_num = gpio_num,
      .mem_block_symbols = LED_CUBE_RMT_MEM_SYMBOLS,
      .resolution_hz = LED_CUBE_RMT_RESOLUTION_HZ,
      .trans_queue_depth = 1,
      .flags =
      {
         .invert_out = false,
         .with_dma = false
      }
   };

   return rmt_new_tx_channel(
      &channel_config,
      &s_tx_channels[channel_index]
   );
}

// Create four RMT TX channels initially assigned to cube layers 0 through 3
static esp_err_t create_tx_channels(void)
{
   for (size_t channel = 0; channel < LED_CUBE_RMT_CHANNEL_COUNT; channel++)
   {
      esp_err_t status = create_tx_channel(
         channel,
         s_layer_gpios[channel]
      );

      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Create encoder that converts WS2812 data bits into RMT pulse timings
static esp_err_t create_ws2812_encoder(void)
{
   rmt_bytes_encoder_config_t encoder_config =
   {
      .bit0 =
      {
         .level0 = 1,
         .duration0 = WS2812_T0H_TICKS,
         .level1 = 0,
         .duration1 = WS2812_T0L_TICKS
      },
      .bit1 =
      {
         .level0 = 1,
         .duration0 = WS2812_T1H_TICKS,
         .level1 = 0,
         .duration1 = WS2812_T1L_TICKS
      },
      .flags =
      {
         .msb_first = true
      }
   };

   return rmt_new_bytes_encoder(
      &encoder_config,
      &s_ws2812_encoder
   );
}

// Enable one RMT TX channel
static esp_err_t enable_tx_channel(size_t channel_index)
{
   if (channel_index >= LED_CUBE_RMT_CHANNEL_COUNT)
      return ESP_ERR_INVALID_ARG;

   return rmt_enable(
      s_tx_channels[channel_index]
   );
}

// Disable one RMT TX channel
static esp_err_t disable_tx_channel(size_t channel_index)
{
   if (channel_index >= LED_CUBE_RMT_CHANNEL_COUNT)
      return ESP_ERR_INVALID_ARG;

   return rmt_disable(
      s_tx_channels[channel_index]
   );
}

// Enable all RMT TX channels used by the cube
static esp_err_t enable_tx_channels(void)
{
   for (size_t channel = 0; channel < LED_CUBE_RMT_CHANNEL_COUNT; channel++)
   {
      esp_err_t status = enable_tx_channel(channel);
      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Disable all RMT TX channels before changing output GPIO assignments
static esp_err_t disable_tx_channels(void)
{
   for (size_t channel = 0; channel < LED_CUBE_RMT_CHANNEL_COUNT; channel++)
   {
      esp_err_t status = disable_tx_channel(channel);
      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Switch one disabled RMT TX channel to another physical layer GPIO
static esp_err_t switch_tx_channel_gpio(
   size_t channel_index,
   size_t layer
)
{
   if (channel_index >= LED_CUBE_RMT_CHANNEL_COUNT)
      return ESP_ERR_INVALID_ARG;

   if (layer >= LED_CUBE_SIZE)
      return ESP_ERR_INVALID_ARG;

   return rmt_tx_switch_gpio(
      s_tx_channels[channel_index],
      s_layer_gpios[layer],
      false // do not invert the output signal
   );
}

// Switch all RMT TX channels to the GPIOs for one four-layer batch
static esp_err_t switch_tx_channels_to_batch(size_t first_layer)
{
   if (first_layer + LED_CUBE_RMT_CHANNEL_COUNT > LED_CUBE_SIZE)
      return ESP_ERR_INVALID_ARG;

   for (size_t channel = 0; channel < LED_CUBE_RMT_CHANNEL_COUNT; channel++)
   {
      esp_err_t status = switch_tx_channel_gpio(
         channel,
         first_layer + channel
      );

      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Convert one logical RGB layer into the GRB byte order expected by WS2812 LEDs
static void prepare_layer_tx_buffer(
   const led_cube_color_t *layer_buffer,
   size_t layer
)
{
   for (size_t pixel = 0; pixel < LED_CUBE_LAYER_PIXELS; pixel++)
   {
      const size_t byte_index = pixel * 3U;

      s_tx_buffer[layer][byte_index] = layer_buffer[pixel].green;
      s_tx_buffer[layer][byte_index + 1U] = layer_buffer[pixel].red;
      s_tx_buffer[layer][byte_index + 2U] = layer_buffer[pixel].blue;
   }
}

// Convert all logical RGB layer data into WS2812 GRB transmission buffers
static void prepare_tx_buffers(
   const led_cube_color_t layer_buffer
      [LED_CUBE_SIZE]
      [LED_CUBE_LAYER_PIXELS]
)
{
   for (size_t layer = 0; layer < LED_CUBE_SIZE; layer++)
   {
      prepare_layer_tx_buffer(
         layer_buffer[layer],
         layer
      );
   }
}

// Transmit one prepared cube layer through one RMT TX channel
static esp_err_t transmit_layer(
   size_t channel_index,
   size_t layer
)
{
   if (channel_index >= LED_CUBE_RMT_CHANNEL_COUNT) return ESP_ERR_INVALID_ARG;

   if (layer >= LED_CUBE_SIZE) return ESP_ERR_INVALID_ARG;

   rmt_transmit_config_t transmit_config = {.loop_count = 0};

   // Starts/queues tx - doesn't mean we are immediately safe to switch channel's GPIO after this
   return rmt_transmit(
      s_tx_channels[channel_index],
      s_ws2812_encoder,
      s_tx_buffer[layer],
      sizeof(s_tx_buffer[layer]),
      &transmit_config
   );
}

// Start parallel transmission for one four-layer cube batch
static esp_err_t transmit_batch(size_t first_layer)
{
   if (first_layer + LED_CUBE_RMT_CHANNEL_COUNT > LED_CUBE_SIZE)
      return ESP_ERR_INVALID_ARG;

   for (size_t channel = 0; channel < LED_CUBE_RMT_CHANNEL_COUNT; channel++)
   {
      esp_err_t status = transmit_layer(
         channel,
         first_layer + channel
      );

      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Wait until all RMT TX channels finish the current transmission batch
static esp_err_t wait_for_batch_complete(void)
{
   for (size_t channel = 0; channel < LED_CUBE_RMT_CHANNEL_COUNT; channel++)
   {
      esp_err_t status = rmt_tx_wait_all_done(
         s_tx_channels[channel],
         -1
      );

      if (status != ESP_OK) return status;
   }

   return ESP_OK;
}

// Reassign all RMT TX channels to the GPIOs for the requested layer batch
static esp_err_t switch_to_batch(size_t first_layer)
{
   esp_err_t status = disable_tx_channels();
   if (status != ESP_OK) return status;

   status = switch_tx_channels_to_batch(first_layer);
   if (status != ESP_OK) return status;

   return enable_tx_channels();
}

// ----------------------------------------
// ------ Public Function Definitions -----
// ----------------------------------------

esp_err_t led_cube_rmt_init(
   const int *layer_gpios,
   size_t layer_count
)
{
   if (layer_gpios == NULL) return ESP_ERR_INVALID_ARG;
   if (layer_count != LED_CUBE_SIZE) return ESP_ERR_INVALID_ARG;
   if (s_initialized) return ESP_OK;

   // Store the physical data GPIO assigned to each cube layer
   memcpy(
      s_layer_gpios,
      layer_gpios,
      sizeof(s_layer_gpios)
   );

   // Create four RMT TX channels initially assigned to layers 0 through 3
   esp_err_t status = create_tx_channels();
   if (status != ESP_OK) return status;

   // Create the shared WS2812 data encoder
   status = create_ws2812_encoder();
   if (status != ESP_OK) return status;

   // Enable all four channels for transmission
   status = enable_tx_channels();
   if (status != ESP_OK) return status;

   s_initialized = true;

   ESP_LOGI(
      LOG_TAG,
      "RMT cube driver initialized with %u TX channels",
      (unsigned)LED_CUBE_RMT_CHANNEL_COUNT
   );

   return ESP_OK;
}

/*
   Physical cube transmission operation.

   Receives the complete logical layer buffer from led_cube.c and transmits
   it to the eight physical WS2812 layers using four reusable RMT TX channels
   in two four-layer batches.

   This function handles physical transmission only. Logical voxel mapping
   and cube state are owned by led_cube.c.
*/
esp_err_t led_cube_rmt_show(
   const led_cube_color_t layer_buffer
      [LED_CUBE_SIZE]
      [LED_CUBE_LAYER_PIXELS]
)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;
   if (layer_buffer == NULL) return ESP_ERR_INVALID_ARG;

   // Convert logical RGB cube state into WS2812 GRB transmission data
   prepare_tx_buffers(layer_buffer);

   // Assign channels to layers 0 through 3
   esp_err_t status = switch_to_batch(0U);
   if (status != ESP_OK) return status;

   // Transmit layers 0 through 3 in parallel
   status = transmit_batch(0U);
   if (status != ESP_OK) return status;

   status = wait_for_batch_complete();
   if (status != ESP_OK) return status;

   // Reassign the same channels to layers 4 through 7
   status = switch_to_batch(LED_CUBE_RMT_CHANNEL_COUNT);
   if (status != ESP_OK) return status;

   // Transmit layers 4 through 7 in parallel
   status = transmit_batch(LED_CUBE_RMT_CHANNEL_COUNT);
   if (status != ESP_OK) return status;

   return wait_for_batch_complete();
}