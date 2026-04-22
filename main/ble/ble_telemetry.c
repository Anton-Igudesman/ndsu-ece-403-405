#include <string.h>

#include "ble_telemetry.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "dsp/spectrum_map.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"

// frame_id, peak_bin, peak_hz_q (hi/lo), spectrum_bands = 12
#define BLE_TELEMETRY_ADV_PAYLOAD_LEN (4U + SPECTRUM_NUM_BANDS)
#define BLE_REFRESH_DIVISOR 8U

// -----------------------------------------
// -------- Function Prototypes ------------
// -----------------------------------------
static void ble_start_advertising(void);

// -----------------------------------------
// -------- Static Globals -----------------
// -----------------------------------------

static const char *LOG_TAG = "ble_telemetry";
static uint8_t s_own_addr_type = 0;
static bool s_ble_started = false;

// Makes up data for BLE advertising packet 
typedef struct
{
   uint8_t frame_id; // Wraps at 255 
   uint8_t peak_bin;
   uint16_t peak_hz_q; // Quantized (integer) Hz
   uint8_t band_level_q[SPECTRUM_NUM_BANDS]; // 0 -> 255 quantized normalized bands
} ble_telemetry_snapshot_t;

static ble_telemetry_snapshot_t s_latest_snapshot = {0};
static bool s_snapshot_valid = false; // Does this contain real data yet

// Update snapshot and helpers
static uint8_t quantize_band_level_to_u8(float normalized_level)
{
   if (normalized_level < 0.0f) normalized_level = 0.0f;
   if (normalized_level > 1.0f) normalized_level = 1.0f;

   // Convert 0->1 value to byte value and cast uint8
   float scaled_level = normalized_level * 255.0f;
   return (uint8_t)(scaled_level + 0.5f);
}

// DSP runtime values become BLE advertisement
// Producer-side trigger for BLE payload refresh
esp_err_t ble_telemetry_update_snapshot(
   uint8_t peak_bin,
   float peak_hz,
   const float *bands_norm,
   size_t num_bands
)
{

   // Validate data from DSP portion
   if (bands_norm == NULL) return ESP_ERR_INVALID_ARG;
   if (num_bands != SPECTRUM_NUM_BANDS) return ESP_ERR_INVALID_ARG;

   s_latest_snapshot.frame_id++;
   s_latest_snapshot.peak_bin = peak_bin;

   // Set absolute min/max values
   if (peak_hz < 0.0f) peak_hz = 0.0f;
   if (peak_hz > 65535.0f) peak_hz = 65535.0f;
   s_latest_snapshot.peak_hz_q = (uint16_t)(peak_hz + 0.5f);

   for (size_t band_index = 0; band_index < SPECTRUM_NUM_BANDS; band_index++)
   {
      s_latest_snapshot.band_level_q[band_index] = 
         quantize_band_level_to_u8(bands_norm[band_index]);
   }

   s_snapshot_valid = true;

   // Verify data before BLE adv packing
   ESP_LOGI(
      LOG_TAG,
      "snapshot frame=%u peak_bin=%u peak_hz_q=%u bands=[%u, %u, %u, %u, %u, %u, %u, %u]",
      (unsigned)s_latest_snapshot.frame_id,
      (unsigned)s_latest_snapshot.peak_bin,
      (unsigned)s_latest_snapshot.peak_hz_q,
      (unsigned)s_latest_snapshot.band_level_q[0],
      (unsigned)s_latest_snapshot.band_level_q[1],
      (unsigned)s_latest_snapshot.band_level_q[2],
      (unsigned)s_latest_snapshot.band_level_q[3],
      (unsigned)s_latest_snapshot.band_level_q[4],
      (unsigned)s_latest_snapshot.band_level_q[5],
      (unsigned)s_latest_snapshot.band_level_q[6],
      (unsigned)s_latest_snapshot.band_level_q[7]
   );

   // NimBLE advertisement throttle 
   if ((s_latest_snapshot.frame_id % BLE_REFRESH_DIVISOR) != 0U) return ESP_OK;

   // Refresh advertising so scanner can read newest telemetry bytes
   // Ignore "not active" stop result

   // Ask GAP layer to stop current advertising procedure
   int stop_status_code = ble_gap_adv_stop(); // Store new result
   if (stop_status_code != 0 && stop_status_code != BLE_HS_EALREADY)
   {
      ESP_LOGW(LOG_TAG, "ble_gap_adv_stop returned status_code=%d", stop_status_code);
   }

   // Rebuild advertisement + scan response from current module state
   ble_start_advertising();

   return ESP_OK;
}

// Header definition for NimBLE defined function
void ble_store_config_init(void);

// Helper function
static esp_err_t ble_set_scan_response_from_snapshot(void)
{
   // No valid snapshot state yet
   if (!s_snapshot_valid) return ESP_OK;

   uint8_t telemetry_payload[BLE_TELEMETRY_ADV_PAYLOAD_LEN] = {0};

   telemetry_payload[0] = s_latest_snapshot.frame_id;
   telemetry_payload[1] = s_latest_snapshot.peak_bin;
   telemetry_payload[2] = (uint8_t)(s_latest_snapshot.peak_hz_q & 0xFFU);
   telemetry_payload[3] = (uint8_t)((s_latest_snapshot.peak_hz_q >> 8) & 0xFFU);

   for (size_t band_index = 0; band_index < SPECTRUM_NUM_BANDS; band_index++)
   {
      telemetry_payload[4 + band_index] = s_latest_snapshot.band_level_q[band_index];
   }

   struct ble_hs_adv_fields scan_response_fields = {0};
   scan_response_fields.mfg_data = telemetry_payload;
   scan_response_fields.mfg_data_len = sizeof(telemetry_payload);

   int status_code = ble_gap_adv_rsp_set_fields(&scan_response_fields);
   if(status_code != 0) return ESP_FAIL;

   return ESP_OK;
}

// Host-side GAP config converted into on-air advertising behavior
static void ble_start_advertising(void)
{
   struct ble_hs_adv_fields adv_fields = {0}; // Advertising data
   struct ble_gap_adv_params adv_params = {0}; // Radio behavior settings

   const char *device_name = ble_svc_gap_device_name();

   // Make beacon discoverable and mark as BLE only
   adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

   // Put complete device name in adv packet
   adv_fields.name = (uint8_t *)device_name;
   adv_fields.name_len = (uint8_t)strlen(device_name);
   adv_fields.name_is_complete = 1;

   int status_code = ble_gap_adv_set_fields(&adv_fields);
   
   if (status_code != 0)
   {
      ESP_LOGE(
         LOG_TAG, 
         "ble_gap_adv_set_fields failed: status_code=%d", 
         status_code);
      return;
   }

   esp_err_t status = ble_set_scan_response_from_snapshot();
   if (status != ESP_OK)
   {
      ESP_LOGE(LOG_TAG, "ble_set_scan_response_from_snapshot failed");
      return;
   }

   // Non-connectable general advertising (simple beacon mode)
   adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
   adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

   // Start radio advertising
   status_code = ble_gap_adv_start(
      s_own_addr_type, 
      NULL,                // Peer - undirected advertising
      BLE_HS_FOREVER, 
      &adv_params,         // Behavior flags
      NULL,                // No per-event handlers in minimal mode
      NULL);

   if (status_code != 0)
   {
      ESP_LOGE(
         LOG_TAG, 
         "ble_gap_adv_start failed: status_code=%d", 
         status_code);
      return;
   }

   ESP_LOGI(LOG_TAG, "advertising started");
}

// BLE initialize -> BLE on-air
static void ble_on_sync(void)
{
   // Select local BLE identity address type
   int status_code = ble_hs_id_infer_auto(
      0, 
      &s_own_addr_type // Write resolved address into module state
   );

   if (status_code != 0)
   {
      ESP_LOGE(
         LOG_TAG, 
         "ble_hs_id_infer_auto failed: status_code=%d", 
         status_code
      );
   }

   ble_start_advertising();
}

static void ble_host_task(void *param)
{
   (void)param;

   ESP_LOGI(LOG_TAG, "nimble host task started");

   // Blocking host loop: processes BLE host events until stack is stopped
   nimble_port_run();

   // Cleanup hook after host loop exits
   vTaskDelete(NULL);
}

static esp_err_t ble_nvs_init(void)
{
   // Initialize nvs flash
   esp_err_t status = nvs_flash_init();

   if (status == ESP_ERR_NVS_NO_FREE_PAGES ||
      status == ESP_ERR_NVS_NEW_VERSION_FOUND)
   {
      status = nvs_flash_erase();
      if (status != ESP_OK) return status;

      status = nvs_flash_init();
      if (status != ESP_OK) return status;
   }

   else if (status != ESP_OK) return status;
   return ESP_OK;
}

static void nimble_host_config_init(void)
{
   // Set host callbacks
   ble_hs_cfg.sync_cb = ble_on_sync;
   ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
   ble_store_config_init();
}

esp_err_t ble_telemetry_init(void)
{
   // NVS flash
   esp_err_t status = ble_nvs_init();
   if (status != ESP_OK) return status;

   // NimBLE host stack init
   status = nimble_port_init();
   if (status != ESP_OK) return status;

   // Initialize GAP service
   ble_svc_gap_init();

   // Set GAP device name 
   int status_code = ble_svc_gap_device_name_set("rtos_ble_led");
   if (status_code != 0) return ESP_FAIL;

   nimble_host_config_init();

   return ESP_OK;
}

esp_err_t ble_telemetry_start(void)
{
   if (s_ble_started) return ESP_OK;

   BaseType_t create_status = xTaskCreate(
      ble_host_task,
      "ble_host_task",
      4096,             // Minimal host loop bring-up
      NULL,
      5,                // High priority app task
      NULL
   );

   // Guard against duplicate host task creation 
   s_ble_started = true;

   return ESP_OK;
}