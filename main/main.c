#include "main.h"
#include "adc.h"
#include "wifi.h"
#include "web/server.h"
#include "rtc_time.h"
#include "nvs_logging.h"
#include "led_status.h"
#include "web/server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "MAIN";

// Boot counter for tracking reboots
static int boot_counter = 0;

// Function prototypes
static esp_err_t load_and_increment_boot_counter(void);

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load and increment boot counter
    ret = load_and_increment_boot_counter();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load boot counter: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Boot #%d", boot_counter);
    }
    
    // Initialize LED status system first for early feedback
    ESP_ERROR_CHECK(led_status_init());
    
    // Initialize networking
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Initialize RTC time management
    ESP_ERROR_CHECK(rtc_init());
    
    // Load WiFi configuration from NVS
    wifi_load_config();
    
    // Create WebSocket data queue for web server
    ws_data_queue = xQueueCreate(10, sizeof(ws_batch_packet_t)); // Queue for 10 WebSocket batch packets
    
    if (ws_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create WebSocket data queue");
        return;
    }

    // Initialize ADC subsystem
    ret = adc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC: %s", esp_err_to_name(ret));
        return;
    }

    // Create WebSocket data transmission task
    xTaskCreate(ws_data_task, "ws_data", 4096*3, NULL, 4, NULL);

    // Start ADC sampling
    ret = adc_start_sampling();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC sampling: %s", esp_err_to_name(ret));
        return;
    }

    // Initialize WiFi and start web server
    wifi_apply_config();
    server = start_webserver();
    if (server) {
        // Subscribe WebSocket to raw ADC data for oscilloscope functionality
        adc_subscribe_raw_values(ws_raw_data_callback);
        
        if (current_wifi_config.mode == 0) {
            ESP_LOGI(TAG, "Web server started in AP mode. Connect to WiFi '%s' and browse to http://192.168.4.1", current_wifi_config.ssid);
        } else {
            ESP_LOGI(TAG, "Web server started in Station mode. Attempting to connect to WiFi '%s'...", current_wifi_config.ssid);
            ESP_LOGI(TAG, "The IP address will be displayed once connected. Then browse to that IP address.");
        }
    }

    // Initialize and start NVS logging
    ret = nvs_logging_init();
    if (ret == ESP_OK) {
        ret = nvs_logging_start();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "NVS logging started - statistics will be logged to flash every second");
        } else {
            ESP_LOGE(TAG, "Failed to start NVS logging: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize NVS logging: %s", esp_err_to_name(ret));
    }

    // Main task just monitors the system
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        // ESP_LOGI(TAG, "System running... Buffer index: %lu", buffer_index);
    }

    // Cleanup (never reached in this implementation)
    adc_cleanup();
}

/*---------------------------------------------------------------
        Boot Counter Management
---------------------------------------------------------------*/
static esp_err_t load_and_increment_boot_counter(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // Open NVS namespace
    err = nvs_open("boot_info", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle for boot counter: %s", esp_err_to_name(err));
        return err;
    }
    
    // Read the boot counter, defaulting to 0 if not found
    size_t required_size = sizeof(boot_counter);
    err = nvs_get_blob(nvs_handle, "boot_count", &boot_counter, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // First boot - initialize counter to 0
        boot_counter = 0;
        ESP_LOGI(TAG, "Boot counter not found in NVS, initializing to 0");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading boot counter from NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Increment the counter
    boot_counter++;
    
    // Write the updated counter back to NVS
    err = nvs_set_blob(nvs_handle, "boot_count", &boot_counter, sizeof(boot_counter));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing boot counter to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Commit the changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing boot counter to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    // Close NVS handle
    nvs_close(nvs_handle);
    
    return ESP_OK;
}

/*---------------------------------------------------------------
        Boot Counter Public API
---------------------------------------------------------------*/
int get_boot_counter(void)
{
    return boot_counter;
}