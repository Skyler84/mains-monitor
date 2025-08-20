#pragma once

#include <stdint.h>

// WiFi AP configuration
#define WIFI_SSID               "mains-monitor"
#define WIFI_PASS               "mains-monitor-password"
#define WIFI_CHANNEL            1
#define MAX_STA_CONN            4

typedef struct {
    char ssid[32];
    char password[64];
    uint8_t mode; // 0 = AP, 1 = Station
} custom_wifi_config_t;

extern custom_wifi_config_t current_wifi_config;

extern void wifi_init_softap(void);
extern void wifi_init_station(void);
extern void wifi_load_config(void);
extern void wifi_save_config(void);
extern void wifi_apply_config(void);
