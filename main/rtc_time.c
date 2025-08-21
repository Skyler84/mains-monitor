#include "rtc_time.h"

#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "RTC_TIME";

// Flag to track if time has been manually set
static bool time_is_set = false;

/*---------------------------------------------------------------
        Private Functions
---------------------------------------------------------------*/

// SNTP time sync notification callback
static void sntp_sync_time_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized via SNTP");
    time_is_set = true;
}

/*---------------------------------------------------------------
        Public API Implementation
---------------------------------------------------------------*/

esp_err_t rtc_init(void)
{
    ESP_LOGI(TAG, "Initializing RTC time management...");
    
    // Set timezone to UTC (can be changed later)
    setenv("TZ", "UTC", 1);
    tzset();
    
    // Initialize SNTP for automatic time sync when WiFi is available
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(sntp_sync_time_cb);
    esp_sntp_init();
    
    ESP_LOGI(TAG, "RTC time management initialized");
    return ESP_OK;
}

esp_err_t rtc_set_time(time_t timestamp)
{
    struct timeval tv;
    tv.tv_sec = timestamp;
    tv.tv_usec = 0;
    
    esp_err_t ret = settimeofday(&tv, NULL);
    if (ret == ESP_OK) {
        time_is_set = true;
        
        char time_str[64];
        rtc_get_time_string(time_str, sizeof(time_str));
        ESP_LOGI(TAG, "Time set to: %s", time_str);
    } else {
        ESP_LOGE(TAG, "Failed to set time");
    }
    
    return ret;
}

time_t rtc_get_time(void)
{
    return time(NULL);
}

esp_err_t rtc_get_time_string(char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    time_t now = rtc_get_time();
    struct tm *timeinfo = gmtime(&now);
    
    if (timeinfo == NULL) {
        strncpy(buffer, "Invalid time", buffer_size - 1);
        buffer[buffer_size - 1] = '\0';
        return ESP_ERR_INVALID_STATE;
    }
    
    strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%S UTC", timeinfo);
    return ESP_OK;
}

bool rtc_is_time_set(void)
{
    // Check if time is set and reasonable (after 2020)
    time_t now = rtc_get_time();
    return time_is_set && (now > 1577836800); // 2020-01-01 00:00:00 UTC
}

esp_err_t rtc_time_to_iso8601(time_t timestamp, char *buffer, size_t buffer_size)
{
    if (buffer == NULL || buffer_size < 25) { // Need at least 24 chars + null terminator
        return ESP_ERR_INVALID_ARG;
    }
    
    struct tm *timeinfo = gmtime(&timestamp);
    if (timeinfo == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    strftime(buffer, buffer_size, "%Y-%m-%dT%H:%M:%SZ", timeinfo);
    return ESP_OK;
}

esp_err_t rtc_iso8601_to_time(const char *iso8601_str, time_t *timestamp)
{
    if (iso8601_str == NULL || timestamp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct tm tm_time = {0};
    
    // Parse ISO8601 format: YYYY-MM-DDTHH:MM:SSZ
    int parsed = sscanf(iso8601_str, "%d-%d-%dT%d:%d:%dZ",
                       &tm_time.tm_year, &tm_time.tm_mon, &tm_time.tm_mday,
                       &tm_time.tm_hour, &tm_time.tm_min, &tm_time.tm_sec);
    
    if (parsed != 6) {
        ESP_LOGE(TAG, "Failed to parse ISO8601 time string: %s", iso8601_str);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Adjust for tm structure (year since 1900, month 0-11)
    tm_time.tm_year -= 1900;
    tm_time.tm_mon -= 1;
    
    // Convert to Unix timestamp
    *timestamp = mktime(&tm_time);
    
    if (*timestamp == -1) {
        ESP_LOGE(TAG, "Invalid time values in ISO8601 string: %s", iso8601_str);
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}
