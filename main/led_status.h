#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------------------------------
        LED Status Indicators
---------------------------------------------------------------*/

// GPIO pin definitions
#define NEOPIXEL_GPIO       8   // Onboard RGB LED (WS2812)
#define USER_LED_GPIO      15   // User controllable LED

// LED status levels
typedef enum {
    LED_STATUS_OK = 0,          // Green - Everything is working normally
    LED_STATUS_WARNING,         // Orange - Minor issues (RTC not set, voltage/freq out of range)
    LED_STATUS_ERROR            // Red - Serious issues (ADC failure, critical errors)
} led_status_t;

// LED warning/error flags
typedef enum {
    LED_FLAG_NONE = 0,
    LED_FLAG_RTC_NOT_SET = (1 << 0),
    LED_FLAG_VOLTAGE_HIGH = (1 << 1),
    LED_FLAG_VOLTAGE_LOW = (1 << 2),
    LED_FLAG_FREQ_HIGH = (1 << 3),
    LED_FLAG_FREQ_LOW = (1 << 4),
    LED_FLAG_ADC_ERROR = (1 << 5),
    LED_FLAG_WIFI_ERROR = (1 << 6),
    LED_FLAG_CRITICAL_ERROR = (1 << 7)
} led_flags_t;

/*---------------------------------------------------------------
        Public API
---------------------------------------------------------------*/

// Initialize LED system
esp_err_t led_status_init(void);

// Set LED status based on flags
esp_err_t led_status_update(uint32_t flags);

// Set specific LED status
esp_err_t led_status_set(led_status_t status);

// Control user LED independently
esp_err_t led_user_set(bool on);

// Get current status
led_status_t led_status_get_current(void);
uint32_t led_status_get_flags(void);

// Blink patterns for diagnostics
esp_err_t led_status_blink(led_status_t status, uint32_t count, uint32_t period_ms);
