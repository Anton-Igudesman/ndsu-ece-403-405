#include "gpio4_scope_test.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *LOG_TAG = "gpio4_scope_test";
static const gpio_num_t TEST_GPIO = GPIO_NUM_4;

static void gpio4_scope_blink_task(void *arg)
{
   (void)arg;
   bool level_high = false;

   while (true)
   {
      level_high = !level_high;
      gpio_set_level(TEST_GPIO, level_high ? 1 : 0);
      vTaskDelay(pdMS_TO_TICKS(500));
   }
}

esp_err_t gpio4_scope_test_start(bool steady_high)
{
   gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << TEST_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
   };

   esp_err_t status = gpio_config(&cfg);
   if (status != ESP_OK) return status;

   if (steady_high)
   {
      status = gpio_set_level(TEST_GPIO, 1);
      if (status == ESP_OK)
      {
         ESP_LOGI(LOG_TAG, "GPIO4 scope test: steady HIGH enabled");
      }
      return status;
   }

   if (xTaskCreate(gpio4_scope_blink_task, "gpio4_scope_blink_task", 2048, NULL, 3, NULL) != pdPASS)
   {
      return ESP_FAIL;
   }

   ESP_LOGI(LOG_TAG, "GPIO4 scope test: 1 Hz square wave enabled");
   return ESP_OK;
}
