#include "app_log.h"
#include "esp_log.h"

void app_log_error(const char *tag, const char *name, esp_err_t status)
{
   if (status != ESP_OK)
   {
      ESP_LOGE(tag, "%s failed: %s", name, esp_err_to_name(status));
   }
}