#include "terminal_session_manager.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// --------------------------------
// ---- Module Scope Definitions---
// --------------------------------

// Track each connected terminal and its pending control request
typedef struct
{
   int socket_fd;
   uint8_t mac[6];
} terminal_client_t;

typedef struct
{
   uint8_t mac[6];
   bool active;
} terminal_request_t;


// Registry for up to five simultaneous terminal connections
static terminal_client_t s_clients[TERMINAL_MAX_CLIENTS];

// Pending requests
static terminal_request_t s_pending_requests[TERMINAL_MAX_CLIENTS];

// Device ownership is tracked independently of individual WebSocket connections
static uint8_t s_owner_mac[6];
static bool s_owner_present = false;

// --------------------------------------
// ----- Private Function Definitions ---
// --------------------------------------

// Reset a pending control request slot
static void clear_request(terminal_request_t *request)
{
   if (request == NULL) return;

   memset(request->mac, 0, sizeof(request->mac));
   request->active = false;
}

// Clear all pending control requests
static void clear_all_pending_requests(void)
{
   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++) clear_request(&s_pending_requests[i]);
}

// Find an existing pending control request for a device
static terminal_request_t *find_pending_request(const uint8_t mac[6])
{
   if (mac == NULL) return NULL;

   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
   {
      if (!s_pending_requests[i].active) continue;

      // Match requests by device, not individual WebSocket connections
      if (memcmp(s_pending_requests[i].mac, mac, sizeof(s_pending_requests[i].mac)) == 0)
         return &s_pending_requests[i];
   }

   return NULL;
}

// Find unused pending control request slot
static terminal_request_t *find_available_request(void)
{
   // Return first unused slot
   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
      if (!s_pending_requests[i].active) return &s_pending_requests[i];

   return NULL;
}

// Find a registered terminal client by socket descriptor
static terminal_client_t *find_client(int socket_fd)
{
   if (socket_fd < 0) return NULL;

   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
   {
      if (s_clients[i].socket_fd == socket_fd) return &s_clients[i];
   }

   return NULL;
}

// Find a registered terminal connection belonging to a device
static terminal_client_t *find_client_by_mac(const uint8_t mac[6])
{
   if (mac == NULL) return NULL;

   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
   {
      if (s_clients[i].socket_fd < 0) continue;

      if (memcmp(s_clients[i].mac, mac, 6) == 0) return &s_clients[i];
   }

   return NULL;
}

// Find an unused terminal connection slot
static terminal_client_t *find_available_slot(void)
{
   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
   {
      if (s_clients[i].socket_fd == -1) return &s_clients[i];
   }

   return NULL;
}

// Reset a terminal client slot to its unused state
static void clear_client(terminal_client_t *client)
{
   if (client == NULL) return;

   client->socket_fd = -1;
   memset(client->mac, 0, sizeof(client->mac));
}

// --------------------------------------
// ----- Public Function Definitions ---
// --------------------------------------

// Initialize terminal connection slots and ownership state
esp_err_t terminal_session_manager_init(void)
{
   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
   {
      clear_client(&s_clients[i]);
      clear_request(&s_pending_requests[i]);
   } 

   // Reset device ownership at startup
   memset(s_owner_mac, 0 , sizeof(s_owner_mac));
   s_owner_present = false;

   return ESP_OK;
}

// Register a terminal connection and enforce the five-client limit
esp_err_t terminal_session_manager_register(int socket_fd, const uint8_t mac[6])
{
   if (socket_fd < 0 || mac == NULL) return ESP_ERR_INVALID_ARG;

   // Prevent duplicate registration of the same connection
   if (find_client(socket_fd) != NULL) return ESP_OK;

   terminal_client_t *client = find_available_slot();
   if (client == NULL) return ESP_ERR_NO_MEM;

   client->socket_fd = socket_fd;
   memcpy(client->mac, mac, sizeof(client->mac));

   // The first connected client automatically receives control
   if (!s_owner_present)
   {
      memcpy(s_owner_mac, mac, sizeof(s_owner_mac));
      s_owner_present = true;
   }

   return ESP_OK;
}

// Remove a terminal connection while preserving device ownership
// Return true when pending-request list changed during disconnection
bool terminal_session_manager_unregister(int socket_fd)
{
   terminal_client_t *client = find_client(socket_fd);
   if (client == NULL) return false;

   uint8_t client_mac[6];
   memcpy(client_mac, client->mac, sizeof(client_mac));

   clear_client(client);

   // Keep ownershp if another tab from the same device remains connected
   if (find_client_by_mac(client_mac) != NULL) return false;

   bool pending_requests_changed = false;

   // Remove pending requests when the device's last connection closes
   terminal_request_t *pending = find_pending_request(client_mac);
   if (pending != NULL)
   {
      clear_request(pending);
      pending_requests_changed = true;
   }

   // Release ownership when the controlling client disconnects
   if (s_owner_present && memcmp(s_owner_mac, client_mac, sizeof(client_mac)) == 0)
   {
      memset(s_owner_mac, 0, sizeof(s_owner_mac));
      s_owner_present = false;

      // Check whether clearing remaining requests changes pending list
      for (size_t i = 0; i < TERMINAL_MAX_CLIENTS; i++)
      {
         if (s_pending_requests[i].active)
         {
            pending_requests_changed = true;
            break;
         }
      }

      // Clear pending control requests after owning device disconnects
      clear_all_pending_requests();
   }

   return pending_requests_changed;
}

// Check whether a registered connection currently owns the terminal
bool terminal_session_manager_is_owner(int socket_fd)
{
   if (!s_owner_present) return false;

   terminal_client_t *client = find_client(socket_fd);
   if (client == NULL) return false;

   // All terminal connections from the owning device share ownership
   return memcmp(client->mac, s_owner_mac, sizeof(s_owner_mac)) == 0;
}

// Return a connected socket belonging to the owning device
int terminal_session_manager_get_owner(void)
{
   if (!s_owner_present) return -1;

   terminal_client_t *client = find_client_by_mac(s_owner_mac);
   if (client == NULL) return -1;

   return client->socket_fd;
}

// Retrieve the device MAC associated with a regitered terminal connection
esp_err_t terminal_session_manager_get_client_mac(
   int socket_fd,
   uint8_t mac[6]
)
{
   if (mac == NULL) return ESP_ERR_INVALID_ARG;

   terminal_client_t *client = find_client(socket_fd);
   if (client == NULL) return ESP_ERR_NOT_FOUND;

   memcpy(mac, client->mac, sizeof(client->mac));

   return ESP_OK;
}

// Request terminal control from the current owner
esp_err_t terminal_session_manager_request_control(int socket_fd)
{
   terminal_client_t *client = find_client(socket_fd);
   if (client == NULL) return ESP_ERR_NOT_FOUND;

   if (terminal_session_manager_is_owner(socket_fd)) return ESP_ERR_INVALID_STATE;

   // Prevent duplicate requests from other tabs on the same device
   if (find_pending_request(client->mac) != NULL) return ESP_ERR_INVALID_STATE;

   // An unowned terminal can be calimed without approval
   if (!s_owner_present)
   {
      memcpy(s_owner_mac, client->mac, sizeof(s_owner_mac));
      s_owner_present = true;
      return ESP_OK;
   }

   // Allocate a pending request
   terminal_request_t *request = find_available_request();
   if (request == NULL) return ESP_ERR_NO_MEM;

   memcpy(request->mac, client->mac, sizeof(request->mac));
   request->active = true;

   return ESP_OK;
}

// Check whether a registered device has a pending terminal control request
bool terminal_session_manager_has_pending_request(int socket_fd)
{
   terminal_client_t *client = find_client(socket_fd);
   if (client == NULL) return false;

   return find_pending_request(client->mac) != NULL;
}

// Retrieve registered terminal connections without exposing the internal registry
size_t terminal_session_manager_get_clients(
   int *socket_fds,
   size_t max_clients
)
{
   if (socket_fds == NULL || max_clients == 0U) return 0U;

   size_t count = 0;

   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS && count < max_clients; i++)
   {
      if (s_clients[i].socket_fd < 0) continue;

      socket_fds[count++] = s_clients[i].socket_fd;
   }

   return count;
}

// Retrieve registered terminal clients waiting for control approval
size_t terminal_session_manager_get_pending_requests(
   int *socket_fds,
   size_t max_clients
)
{
   if (socket_fds == NULL || max_clients == 0U) return 0U;

   size_t count = 0;

   for (size_t i = 0; i < TERMINAL_MAX_CLIENTS && count < max_clients; i++)
   {
      if (!s_pending_requests[i].active) continue;

      // Resolve each device request to one of its connected terminal sockets
      terminal_client_t *client = find_client_by_mac(s_pending_requests[i].mac);
      if (client == NULL) continue;

      socket_fds[count++] = client->socket_fd;
   }

   return count;
}

// Transfer terminal ownership after the current owner approves a request
esp_err_t terminal_session_manager_grant_control(
   int owner_fd,
   int requester_fd
)
{
   // Only the current owner can approve a transfer
   if (!terminal_session_manager_is_owner(owner_fd)) return ESP_ERR_INVALID_STATE;

   terminal_client_t *requester = find_client(requester_fd);
   if (requester == NULL) return ESP_ERR_NOT_FOUND;

   // Only grant control to a client with a pending request
   terminal_request_t *pending = find_pending_request(requester->mac);
   if (pending == NULL) return ESP_ERR_INVALID_STATE;

   // Transfer ownership to requesting device and fulfill request
   memcpy(s_owner_mac, requester->mac, sizeof(s_owner_mac));
   s_owner_present = true;

   clear_request(pending);

   return ESP_OK;
}

// Deny a pending ownership transfer
esp_err_t terminal_session_manager_deny_control(
   int owner_fd,
   int requester_fd
)
{
   if (!terminal_session_manager_is_owner(owner_fd)) return ESP_ERR_INVALID_STATE;

   terminal_client_t *requester = find_client(requester_fd);
   if (requester == NULL) return ESP_ERR_NOT_FOUND;

   // Find pending request associated with the requesting device
   terminal_request_t *pending = find_pending_request(requester->mac);
   if (pending == NULL) return ESP_ERR_INVALID_STATE;

   clear_request(pending);

   return ESP_OK;
}

// Voluntarily release terminal ownership
esp_err_t terminal_session_manager_release_control(int socket_fd)
{
   if (!terminal_session_manager_is_owner(socket_fd)) return ESP_ERR_INVALID_STATE;

   // Release ownership at device level
   memset(s_owner_mac, 0, sizeof(s_owner_mac));
   s_owner_present = false;

   // Pending control requests are cleared once ownership is released
   clear_all_pending_requests();

   return ESP_OK;
}

// Transfer terminal ownership immediately to an administrator
esp_err_t terminal_session_manager_admin_takeover(int socket_fd)
{
   terminal_client_t *client = find_client(socket_fd);
   if (client == NULL) return ESP_ERR_NOT_FOUND;

   // Transfer ownership to administrator's device
   memcpy(s_owner_mac, client->mac, sizeof(s_owner_mac));
   s_owner_present = true;

   // Fulfill any pending request from the administrator's device
   terminal_request_t *pending = find_pending_request(client->mac);
   if (pending != NULL) clear_request(pending);

   return ESP_OK;
}