#include "wifi.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "WIFI_CONFIG";

/*---------------------------------------------------------------
        WiFi Event Handler
---------------------------------------------------------------*/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected from WiFi, attempting to reconnect...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected! IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

// WiFi configuration
custom_wifi_config_t current_wifi_config = {
    .ssid = WIFI_SSID,
    .password = WIFI_PASS,
    .mode = 0 // Default to AP mode
};



/*---------------------------------------------------------------
        WiFi Configuration Functions
---------------------------------------------------------------*/
void wifi_load_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &nvs_handle);
    
    if (err == ESP_OK) {
        size_t required_size;
        
        // Load SSID
        required_size = sizeof(current_wifi_config.ssid);
        err = nvs_get_str(nvs_handle, "ssid", current_wifi_config.ssid, &required_size);
        if (err != ESP_OK) {
            strcpy(current_wifi_config.ssid, WIFI_SSID);
        }
        
        // Load password
        required_size = sizeof(current_wifi_config.password);
        err = nvs_get_str(nvs_handle, "password", current_wifi_config.password, &required_size);
        if (err != ESP_OK) {
            strcpy(current_wifi_config.password, WIFI_PASS);
        }
        
        // Load mode
        err = nvs_get_u8(nvs_handle, "mode", &current_wifi_config.mode);
        if (err != ESP_OK) {
            current_wifi_config.mode = 0; // Default to AP mode
        }
        
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "WiFi config loaded: SSID=%s, Mode=%s", 
                 current_wifi_config.ssid, 
                 current_wifi_config.mode == 0 ? "AP" : "Station");
    } else {
        ESP_LOGI(TAG, "WiFi config not found, using defaults");
        strcpy(current_wifi_config.ssid, WIFI_SSID);
        strcpy(current_wifi_config.password, WIFI_PASS);
        current_wifi_config.mode = 0;
    }
}

void wifi_save_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        nvs_set_str(nvs_handle, "ssid", current_wifi_config.ssid);
        nvs_set_str(nvs_handle, "password", current_wifi_config.password);
        nvs_set_u8(nvs_handle, "mode", current_wifi_config.mode);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "WiFi config saved");
    } else {
        ESP_LOGE(TAG, "Failed to save WiFi config");
    }
}

void wifi_apply_config(void)
{
    if (current_wifi_config.mode == 0) {
        wifi_init_softap();
    } else {
        wifi_init_station();
    }
}

/*---------------------------------------------------------------
        WiFi Access Point Setup
---------------------------------------------------------------*/
void wifi_init_softap(void)
{
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(current_wifi_config.ssid),
            .channel = WIFI_CHANNEL,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    strcpy((char*)wifi_config.ap.ssid, current_wifi_config.ssid);
    strcpy((char*)wifi_config.ap.password, current_wifi_config.password);
    
    if (strlen(current_wifi_config.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID: %s", current_wifi_config.ssid);
}

void wifi_init_station(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register WiFi event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    strcpy((char*)wifi_config.sta.ssid, current_wifi_config.ssid);
    strcpy((char*)wifi_config.sta.password, current_wifi_config.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi Station mode started. Connecting to SSID: %s", current_wifi_config.ssid);
}
