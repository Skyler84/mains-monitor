#pragma once

#include <time.h>
#include <sys/time.h>
#include "esp_err.h"
#include <stdbool.h>

/*---------------------------------------------------------------
        RTC Time Management
---------------------------------------------------------------*/

// Initialize RTC system
esp_err_t rtc_init(void);

// Set current time (usually from web interface)
esp_err_t rtc_set_time(time_t timestamp);

// Get current time as Unix timestamp
time_t rtc_get_time(void);

// Get current time as formatted string
esp_err_t rtc_get_time_string(char *buffer, size_t buffer_size);

// Check if time has been set (not just default)
bool rtc_is_time_set(void);

// Convert time_t to ISO8601 string for JSON
esp_err_t rtc_time_to_iso8601(time_t timestamp, char *buffer, size_t buffer_size);

// Parse ISO8601 string to time_t (for web API)
esp_err_t rtc_iso8601_to_time(const char *iso8601_str, time_t *timestamp);
