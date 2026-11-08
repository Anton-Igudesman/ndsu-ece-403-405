#include "web_terminal.h"

#include <stdint.h>
#include <string.h>

#include "led_controller.h"
#include "led_cube.h"

// ---------------------------------------
// ----- Private Function Definitions ----
// ---------------------------------------

// Read an unsigned integer parameter within the specified range
static esp_err_t get_uint_param(
   const cJSON *params,
   const char *name,
   uint32_t max_value,
   uint32_t *value
)
{
   if (params == NULL || name == NULL || value == NULL) return ESP_ERR_INVALID_ARG;

   const cJSON *item = cJSON_GetObjectItemCaseSensitive(params, name);

   if (!cJSON_IsNumber(item)) return ESP_ERR_INVALID_ARG;

   double number = item->valuedouble;

   // Reject negative, fractional, and out-of-range values
   if ( number < 0.0 ||
      number > max_value ||
      number != (double)(uint32_t)number) return ESP_ERR_INVALID_ARG;
   
   *value = (uint32_t)number;

   return ESP_OK;
}

// Validate RGB values and fill the cube framebuffer
static esp_err_t execute_fill(const cJSON *params)
{
   uint32_t red;
   uint32_t green;
   uint32_t blue;

   if (get_uint_param(params, "r", 255U, &red) != ESP_OK ||
      get_uint_param(params, "g", 255U, &green) != ESP_OK ||
      get_uint_param(params, "b", 255U, &blue) != ESP_OK) return ESP_ERR_INVALID_ARG;

   return led_cube_fill(
      (uint8_t)red,
      (uint8_t)green,
      (uint8_t)blue
   );
}

// Validate coordinates and RGB values, then update one voxel
static esp_err_t execute_set_pixel(const cJSON *params)
{
   uint32_t x, y, z;
   uint32_t red, green, blue;

   // Coords must be within the 8x8x8 cube - mirrors the client side validation
   if (get_uint_param(params, "x", LED_CUBE_SIZE - 1U, &x) != ESP_OK ||
      get_uint_param(params, "y", LED_CUBE_SIZE - 1U, &y) != ESP_OK ||
      get_uint_param(params, "z", LED_CUBE_SIZE - 1U, &z) != ESP_OK ||
      get_uint_param(params, "r", 255U, &red) != ESP_OK ||
      get_uint_param(params, "g", 255U, &green) != ESP_OK ||
      get_uint_param(params, "b", 255U, &blue) != ESP_OK) return ESP_ERR_INVALID_ARG;

   return led_cube_set_voxel(
      (size_t)x,
      (size_t)y,
      (size_t)z,
      (uint8_t)red,
      (uint8_t)green,
      (uint8_t)blue
   );
}
// ------------------------------------
// --- Public Function Definitions ----
// ------------------------------------

esp_err_t web_terminal_execute(const cJSON *command)
{
   if (!cJSON_IsObject(command)) return ESP_ERR_INVALID_ARG;

   // Handle Command Mode activation before checking the active mode
   const cJSON *action = cJSON_GetObjectItemCaseSensitive(command, "action");

   if (cJSON_IsString(action) && strcmp(action->valuestring, "set_mode") == 0)
   {
      const cJSON *mode = cJSON_GetObjectItemCaseSensitive(command, "mode");

      if (!cJSON_IsString(mode) || strcmp(mode->valuestring, "command") != 0) return ESP_ERR_INVALID_ARG;

      return led_controller_set_mode(LED_MODE_COMMAND);
   }

   // Allow the frontend to query the current operating mode
   if (cJSON_IsString(action) && strcmp(action->valuestring, "get_mode") == 0)
   {
      return led_controller_get_mode() == LED_MODE_COMMAND
         ? ESP_OK
         : ESP_ERR_INVALID_STATE;
   }

   // Only execute terminal commands while Command Mode is active
   if (led_controller_get_mode() != LED_MODE_COMMAND) return ESP_ERR_INVALID_STATE;

   const cJSON *name = cJSON_GetObjectItemCaseSensitive(command, "command");
   const cJSON *params = cJSON_GetObjectItemCaseSensitive(command, "params");

   if (!cJSON_IsString(name) || !cJSON_IsObject(params)) return ESP_ERR_INVALID_ARG;

   // Map terminal commands to hardware functions
   if (strcmp(name->valuestring, "clear") == 0) return led_cube_clear();
   if (strcmp(name->valuestring, "fill") == 0) return execute_fill(params);
   if (strcmp(name->valuestring, "set_pixel") == 0) return execute_set_pixel(params);
   if (strcmp(name->valuestring, "show") == 0) return led_cube_show();

   return ESP_ERR_NOT_SUPPORTED;
}