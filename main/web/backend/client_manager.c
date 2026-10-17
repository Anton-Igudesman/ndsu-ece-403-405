#include "client_manager.h"

#include <string.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

// -----------------------------------------------------------
// ---------------- Module Scope Definitions -----------------
// -----------------------------------------------------------

static const char *LOG_TAG = "client_manager";

static client_record_t s_clients[CLIENT_MANAGER_MAX_CLIENTS];
static size_t s_client_count = 0;

// -----------------------------------------------------------
// ---------------- Private Function Definitions -------------
// -----------------------------------------------------------

static client_record_t *find_client_by_ip(const esp_ip4_addr_t *ip)
{
   if (ip == NULL) return NULL;

   for (size_t client = 0; client < s_client_count; client++)
      if (s_clients[client].ip.addr == ip->addr) return &s_clients[client];

   return NULL;
}


// Find a connected client by MAC address.
static client_record_t *find_client_by_mac(const uint8_t mac[6])
{
   if (mac == NULL) return NULL;

   for (size_t client = 0; client < s_client_count; client++)
   {
      if (memcmp(
            s_clients[client].mac,
            mac,
            sizeof(s_clients[client].mac)
         ) == 0) return &s_clients[client];
   }

   return NULL;
}

// Update the authorization state of a connected client
static esp_err_t set_client_auth_state(
   const uint8_t mac[6],
   client_auth_state_t auth_state
)
{
   if (mac == NULL) return ESP_ERR_INVALID_ARG;

   client_record_t *client = find_client_by_mac(mac);

   if (client == NULL) return ESP_ERR_NOT_FOUND;

   client->auth_state = auth_state;

   return ESP_OK;
}

// -----------------------------------------------------------
// ---------------- Public Function Definitions --------------
// -----------------------------------------------------------

// Request controller access for a connected client
esp_err_t client_manager_request_controller(const uint8_t mac[6])
{
   client_auth_state_t auth_state;

   esp_err_t status = client_manager_get_auth_state(
      mac,
      &auth_state
   );

   if (status != ESP_OK) return status;

   if (auth_state != CLIENT_AUTH_NONE)
      return ESP_ERR_INVALID_STATE;

   return client_manager_set_pending(mac);
}

// Mark a connected client as waiting for administrator approval.
esp_err_t client_manager_set_pending(const uint8_t mac[6])
{
   esp_err_t status = set_client_auth_state(
      mac,
      CLIENT_AUTH_PENDING
   );

   if (status != ESP_OK) return status;

   ESP_LOGI(
      LOG_TAG,
      "Client %02X:%02X:%02X:%02X:%02X:%02X awaiting controller approval",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]
   );

   return ESP_OK;
}

// Authorize a connected client to control the LED cube
esp_err_t client_manager_set_controller(const uint8_t mac[6])
{
   esp_err_t status = set_client_auth_state(
      mac,
      CLIENT_AUTH_CONTROLLER
   );

   if (status != ESP_OK) return status;

   ESP_LOGI(
      LOG_TAG,
      "Client %02X:%02X:%02X:%02X:%02X:%02X authorized as controller",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]
   );

   return ESP_OK;
}

// Check whether a connected client is authorized to control the LED cube
bool client_manager_is_authorized(const uint8_t mac[6])
{
   client_record_t *client = find_client_by_mac(mac);

   if (client == NULL) return false;

   return client->auth_state == CLIENT_AUTH_CONTROLLER ||
      client->auth_state == CLIENT_AUTH_ADMIN;
}

// Copy the connected-client registry to the caller
esp_err_t client_manager_get_clients(
   client_record_t *clients,
   size_t max_clients,
   size_t *client_count
)
{
   if (clients == NULL || client_count == NULL) return ESP_ERR_INVALID_ARG;

   *client_count = 0;

   if (max_clients < s_client_count) return ESP_ERR_INVALID_SIZE;

   memcpy(
      clients,
      s_clients,
      s_client_count * sizeof(client_record_t)
   );

   *client_count = s_client_count;

   return ESP_OK;
}

// Approve a pending controller access request
esp_err_t client_manager_approve_controller(const uint8_t mac[6])
{
   client_auth_state_t auth_state;

   esp_err_t status = client_manager_get_auth_state(
      mac,
      &auth_state
   );

   if (status != ESP_OK) return status;

   if (auth_state != CLIENT_AUTH_PENDING)
      return ESP_ERR_INVALID_STATE;

   return client_manager_set_controller(mac);
}

// Deny a pending controller access request
esp_err_t client_manager_deny_controller(const uint8_t mac[6])
{
   client_auth_state_t auth_state;

   esp_err_t status = client_manager_get_auth_state(
      mac,
      &auth_state
   );

   if (status != ESP_OK) return status;

   if (auth_state != CLIENT_AUTH_PENDING)
      return ESP_ERR_INVALID_STATE;

   return client_manager_set_none(mac);
}

/*
   Get the MAC address associated with a client's current IP address.

   This allows network-facing code to identify the client by MAC
   without exposing the client manager's internal registry.
*/
esp_err_t client_manager_get_mac_by_ip(
   const esp_ip4_addr_t *ip,
   uint8_t mac[6]
)
{
   if (ip == NULL || mac == NULL)
      return ESP_ERR_INVALID_ARG;

   client_record_t *client = find_client_by_ip(ip);

   if (client == NULL)
      return ESP_ERR_NOT_FOUND;

   // Copy the MAC out of the internal client record for the caller.
   memcpy(
      mac,
      client->mac,
      sizeof(client->mac)
   );

   return ESP_OK;
}

// Check whether a connected client has authenticated as an administrator.
bool client_manager_is_admin(const uint8_t mac[6])
{
   client_record_t *client = find_client_by_mac(mac);

   if (client == NULL) return false;

   return client->auth_state == CLIENT_AUTH_ADMIN;
}

// Authenticate a connected client as an administrator
esp_err_t client_manager_set_admin(const uint8_t mac[6])
{
   esp_err_t status = set_client_auth_state(
      mac,
      CLIENT_AUTH_ADMIN
   );

   if (status != ESP_OK) return status;

   ESP_LOGI(
      LOG_TAG,
      "Client %02X:%02X:%02X:%02X:%02X:%02X authenticated as administrator",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]
   );

   return ESP_OK;
}

esp_err_t client_manager_set_admin_by_ip(const esp_ip4_addr_t *ip)
{

   if (ip == NULL) return ESP_ERR_INVALID_ARG;

   client_record_t *client = find_client_by_ip(ip);

   if (client == NULL) return ESP_ERR_NOT_FOUND;

   return client_manager_set_admin(client->mac);
}

// Revoke authorization for a connected client
esp_err_t client_manager_set_none(const uint8_t mac[6])
{
   esp_err_t status = set_client_auth_state(
      mac,
      CLIENT_AUTH_NONE
   );

   if (status != ESP_OK) return status;

   ESP_LOGI(
      LOG_TAG,
      "Client %02X:%02X:%02X:%02X:%02X:%02X authorization revoked",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]
   );

   return ESP_OK;
}

esp_err_t client_manager_init(void)
{
   memset(s_clients, 0, sizeof(s_clients));

   s_client_count = 0;

   ESP_LOGI(
      LOG_TAG,
      "Client manager initialized"
   );
   return ESP_OK;
}

// Get the authorization state of a connected client
esp_err_t client_manager_get_auth_state(
   const uint8_t mac[6],
   client_auth_state_t *auth_state
)
{
   if (mac == NULL || auth_state == NULL) return ESP_ERR_INVALID_ARG;

   client_record_t *client = find_client_by_mac(mac);

   if (client == NULL) return ESP_ERR_NOT_FOUND;

   *auth_state = client->auth_state;

   return ESP_OK;
}

// Get MAC addresses of all stations currently associated with the SoftAP
esp_err_t client_manager_sync_connected_clients(void)
{
   wifi_sta_list_t station_list = {0};

   esp_err_t status = esp_wifi_ap_get_sta_list(&station_list);

   if (status != ESP_OK)
   {
      ESP_LOGE(
         LOG_TAG,
         "Failed to get connected WiFi stations: %s",
         esp_err_to_name(status)
      );

      return status;
   }

   size_t client_count = station_list.num;

   if (client_count > CLIENT_MANAGER_MAX_CLIENTS) client_count = CLIENT_MANAGER_MAX_CLIENTS;

   // Use the struct to associate each station MAC with IPv4 address assigned by DHCP
   esp_netif_pair_mac_ip_t network_list[CLIENT_MANAGER_MAX_CLIENTS] = {0};

   for (size_t client = 0; client < client_count; client++)
   {
      memcpy(
         network_list[client].mac,
         station_list.sta[client].mac,
         sizeof(network_list[client].mac)
      );
   }

   esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

   if (ap_netif == NULL)
   {
      ESP_LOGE(
         LOG_TAG,
         "Unable to get SoftAP network interface"
      );

      return ESP_ERR_INVALID_STATE;
   }

   // Resolve each connected station MAC to its DHCP-assigned IPv4 address
   status = esp_netif_dhcps_get_clients_by_mac(
      ap_netif,
      (int)client_count,
      network_list
   );

   if (status != ESP_OK)
   {
      ESP_LOGE(
         LOG_TAG,
         "Failed to resolve client IP addresses: %s",
         esp_err_to_name(status)
      );

      return status;
   }

   client_record_t updated_clients[CLIENT_MANAGER_MAX_CLIENTS] = {0};

   for (size_t client = 0; client < client_count; client++)
   {
      client_auth_state_t auth_state = CLIENT_AUTH_NONE;

      // Preserve authorization for a device that is still connected
      for (size_t existing = 0; existing < s_client_count; existing++)
      {
         if (memcmp(
            s_clients[existing].mac,
            network_list[client].mac,
            sizeof(s_clients[existing].mac)
         ) == 0)
         {
            auth_state = s_clients[existing].auth_state;
            break;
         }
      }

      memcpy(updated_clients[client].mac,
      network_list[client].mac,
      sizeof(updated_clients[client].mac));

      updated_clients[client].ip = network_list[client].ip;
      updated_clients[client].auth_state = auth_state;

      ESP_LOGI(
         LOG_TAG,
         "Client %u: MAC=%02X:%02X:%02X:%02X:%02X:%02X IP=" IPSTR,
         (unsigned)client,
         updated_clients[client].mac[0],
         updated_clients[client].mac[1],
         updated_clients[client].mac[2],
         updated_clients[client].mac[3],
         updated_clients[client].mac[4],
         updated_clients[client].mac[5],
         IP2STR(&updated_clients[client].ip)
      );
   }

   // Replace the old registry with the current connected-client list
   memcpy(
      s_clients,
      updated_clients,
      sizeof(s_clients)
   );

   s_client_count = client_count;

   return ESP_OK;
}