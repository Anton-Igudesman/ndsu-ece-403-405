#ifndef CLIENT_MANAGER_H
#define CLIENT_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_netif_ip_addr.h"

typedef enum{
   CLIENT_AUTH_NONE = 0,
   CLIENT_AUTH_PENDING,
   CLIENT_AUTH_CONTROLLER,
   CLIENT_AUTH_ADMIN
} client_auth_state_t;

typedef struct
{
   uint8_t mac[6];
   esp_ip4_addr_t ip;
   client_auth_state_t auth_state;
} client_record_t;

esp_err_t client_manager_init(void);
esp_err_t client_manager_sync_connected_clients(void);

esp_err_t client_manager_set_admin(const uint8_t mac[6]);
esp_err_t client_manager_set_admin_by_ip(const esp_ip4_addr_t *ip);

esp_err_t client_manager_get_mac_by_ip(
   const esp_ip4_addr_t *ip,
   uint8_t mac[6]
);

esp_err_t client_manager_set_controller(const uint8_t mac[6]);
bool client_manager_is_admin(const uint8_t mac[6]);
bool client_manager_is_authorized(const uint8_t mac[6]);

esp_err_t client_manager_get_clients(
   client_record_t *clients,
   size_t max_clients,
   size_t *client_count
);

#endif