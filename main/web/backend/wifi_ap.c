#include "wifi_ap.h"
#include "web_server.h"

#include <stdbool.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

// ------------------------
// ---- WiFi AP Settings --
// ------------------------

#define WIFI_AP_SSID "LED-Cube"
#define WIFI_AP_PASSWORD "admin123"
#define WIFI_AP_CHANNEL 1U
#define WIFI_AP_MAX_CONNECTIONS 6U

// -------------------------------
// --- Module Scope Definitions --
// -------------------------------

static const char *LOG_TAG = "wifi_ap";
static esp_netif_t *s_ap_netif = NULL;
static bool s_initialized = false;
static bool s_started = false;

// ----------------------------------------
// ----- Private Function Definitions -----
// ----------------------------------------

/*
   Handle WiFi access point client events.

   Client connection and disconnection events notify the web server that
   the connected-client list changed. IP assignment is also monitored so
   the dashboard refreshes after DHCP has assigned the client's address.
*/
static void wifi_ap_event_handler(
   void *handler_arg,
   esp_event_base_t event_base,
   int32_t event_id,
   void *event_data
)
{
   (void)handler_arg;
   (void)event_data;

   if (event_base == WIFI_EVENT)
   {
      if (event_id == WIFI_EVENT_AP_STACONNECTED)
      {
         ESP_LOGI(
            LOG_TAG,
            "WiFi client connected"
         );

         esp_err_t status = web_server_notify_clients_changed();

         if (status != ESP_OK)
         {
            ESP_LOGW(
               LOG_TAG,
               "Failed to notify dashboard: %s",
               esp_err_to_name(status)
            );
         }
      }

      else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
      {
         ESP_LOGI(
            LOG_TAG,
            "WiFi client disconnected"
         );

         esp_err_t status = web_server_notify_clients_changed();

         if (status != ESP_OK)
         {
            ESP_LOGW(
               LOG_TAG,
               "Failed to notify dashboard: %s",
               esp_err_to_name(status)
            );
         }
      }

      return;
   }

   if (event_base == IP_EVENT &&
      event_id == IP_EVENT_AP_STAIPASSIGNED)
   {
      ESP_LOGI(
         LOG_TAG,
         "WiFi client assigned IP address"
      );

      esp_err_t status = web_server_notify_clients_changed();

      if (status != ESP_OK)
      {
         ESP_LOGW(
            LOG_TAG,
            "Failed to notify dashboard: %s",
            esp_err_to_name(status)
         );
      }
   }
}

// Register WiFi and IP events used to monitor access point clients
static esp_err_t register_event_handlers(void)
{
   esp_err_t status = esp_event_handler_register(
      WIFI_EVENT,
      WIFI_EVENT_AP_STACONNECTED,
      wifi_ap_event_handler,
      NULL
   );

   if (status != ESP_OK) return status;

   status = esp_event_handler_register(
      WIFI_EVENT,
      WIFI_EVENT_AP_STADISCONNECTED,
      wifi_ap_event_handler,
      NULL
   );

   if (status != ESP_OK) return status;

   status = esp_event_handler_register(
      IP_EVENT,
      IP_EVENT_AP_STAIPASSIGNED,
      wifi_ap_event_handler,
      NULL
   );

   if (status != ESP_OK) return status;

   return ESP_OK;
}

// Initialize the TCP/IP network itnerface layer used by WiFi AP
static esp_err_t initialize_network_interface(void)
{
   esp_err_t status = esp_netif_init();

   if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) return status;
   return ESP_OK;
}

// Create default event loop used by WiFi driver
static esp_err_t initialize_event_loop(void)
{
   esp_err_t status = esp_event_loop_create_default();

   if (status != ESP_OK && status != ESP_ERR_INVALID_STATE) return status;
   return ESP_OK;
}

// Create network interface used by WiFi AP
static esp_err_t create_ap_network_interface(void)
{
   s_ap_netif = esp_netif_create_default_wifi_ap();

   if (s_ap_netif == NULL) return ESP_FAIL;
   return ESP_OK;
}

// Initialize ESP-IDF WiFi driver with default config
static esp_err_t initialize_wifi_driver(void)
{
   wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
   return esp_wifi_init(&wifi_config); // initialize wifi with default config object
}

// Configure AP network and connection settings
static esp_err_t configure_access_point(void)
{
   wifi_config_t ap_config = {0};

   // Copy SSID and password settings into fixed-size WiFi config buffers
   memcpy(
      ap_config.ap.ssid,
      WIFI_AP_SSID,
      strlen(WIFI_AP_SSID)
   );

   memcpy(
      ap_config.ap.password,
      WIFI_AP_PASSWORD,
      strlen(WIFI_AP_PASSWORD)
   );

   ap_config.ap.ssid_len = strlen(WIFI_AP_SSID);
   ap_config.ap.channel = WIFI_AP_CHANNEL;
   ap_config.ap.max_connection = WIFI_AP_MAX_CONNECTIONS;
   ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;

   // Apply AP config to WiFi driver
   return esp_wifi_set_config(
      WIFI_IF_AP,
      &ap_config
   );
}

// ----------------------------------------
// ------ Public Function Definitions -----
// ----------------------------------------

esp_err_t wifi_ap_init(void)
{
   if (s_initialized) return ESP_OK;

   esp_err_t status = initialize_network_interface();
   if (status != ESP_OK) return status;

   status = initialize_event_loop();
   if (status != ESP_OK) return status;

   // Register access point client monitoring events
   status = register_event_handlers();
   if (status != ESP_OK) return status;

   status = create_ap_network_interface();
   if (status != ESP_OK) return status;

   status = initialize_wifi_driver();
   if (status != ESP_OK) return status;

   // Configure WiFi driver to operate as AP
   status = esp_wifi_set_mode(WIFI_MODE_AP);
   if (status != ESP_OK) return status;

   status = configure_access_point();
   if (status != ESP_OK) return status;

   s_initialized = true;

   return ESP_OK;
}

esp_err_t wifi_ap_start(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;
   if (s_started) return ESP_OK;

   esp_err_t status = esp_wifi_start();
   if (status != ESP_OK) return status;

   s_started = true;

   ESP_LOGI(
      LOG_TAG,
      "WiFi AP started: SSID=%s",
      WIFI_AP_SSID
   );

   return ESP_OK;
}

esp_err_t wifi_ap_stop(void)
{
   if (!s_initialized) return ESP_ERR_INVALID_STATE;
   if (!s_started) return ESP_OK;

   esp_err_t status = esp_wifi_stop();
   if (status != ESP_OK) return status;

   s_started = false;

   ESP_LOGI(
      LOG_TAG,
      "WiFi AP stopped"
   );

   return ESP_OK;
}