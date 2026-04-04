#ifndef LED_PROTOCOL_H
#define LED_PROTOCOL_H 

#include <stdint.h>

// Definition for branch logic in led_task
typedef enum
{
   LED_MODE_BLINK,
   LED_MODE_FADE,
   LED_MODE_BREATHE,
   LED_MODE_ON_SOLID,
   LED_MODE_AUDIO_EQ,
   LED_MODE_OFF
} led_mode_t;

// Definition for mode a blink commands
typedef enum 
{
   LED_CMD_SET_MODE,
   LED_CMD_SET_PERIOD_MS
} led_cmd_type_t;

// Struct uses led_cmd_type_t enum for mode
typedef struct
{
   led_cmd_type_t type;
   led_mode_t mode; 
   uint32_t period_ms;
} led_cmd_t;

#endif

