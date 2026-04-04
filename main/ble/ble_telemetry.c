#include <string.h>

#include "ble_telemetry.h"

#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "store/config/ble_store_config.h"

static const char *LOG_TAG = "ble_telemetry";
static uint8_t s_own_addr_type = 0;
static bool s_ble_started = false;

// Header definition for NimBLE defined function
void ble_store_config_init(void);

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