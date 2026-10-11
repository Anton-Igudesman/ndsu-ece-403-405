#include "app_system.h"

#include "nvs_flash.h"

// Initialize shared non-volatile storage used by system services
esp_err_t app_system_init_nvs(void)
{
   esp_err_t status = nvs_flash_init();

   // Recreate NVS if the existing partition cannot be used
   if (status == ESP_ERR_NVS_NO_FREE_PAGES ||
      status == ESP_ERR_NVS_NEW_VERSION_FOUND)
   {
      status = nvs_flash_erase();
      if (status != ESP_OK) return status;

      status = nvs_flash_init();
   }

   return status;
}