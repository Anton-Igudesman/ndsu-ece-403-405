#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Maximum number of simultaneous terminal WebSocket connections
#define TERMINAL_MAX_CLIENTS 5U

// Initialize terminal session tracking
esp_err_t terminal_session_manager_init(void);

// Register or remove a terminal connection
esp_err_t terminal_session_manager_register(int socket_fd, const uint8_t mac[6]);
void terminal_session_manager_unregister(int socket_fd);

// Check whether a connection currently owns the terminal
bool terminal_session_manager_is_owner(int socket_fd);

// Retrieve the current terminal owner
int terminal_session_manager_get_owner(void);

// Retrieve the MAC address of a registered terminal connection
esp_err_t terminal_session_manager_get_client_mac(int socket_fd, uint8_t mac[6]);

// Retrieve registered terminal connections
size_t terminal_session_manager_get_clients(
   int *socket_fds,
   size_t max_clients
);

// Retrieve terminal connections with pending control requests
size_t terminal_session_manager_get_pending_requests(
   int *socket_fds,
   size_t max_clients
);

// Request terminal control from the current owner
esp_err_t terminal_session_manager_request_control(int socket_fd);

// Grant or deny a pending control request
esp_err_t terminal_session_manager_grant_control(
   int owner_fd,
   int requester_fd
);

esp_err_t terminal_session_manager_deny_control(
   int owner_fd,
   int requester_fd
);

// Voluntarily release terminal control
esp_err_t terminal_session_manager_release_control(int socket_fd);

// Allow an authenticated administrator to take control immediately
esp_err_t terminal_session_manager_admin_takeover(int socket_fd);
