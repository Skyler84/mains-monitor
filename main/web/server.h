#pragma once

#include "main.h"

#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"



// WebSocket data packet structure
typedef struct {
    float voltage_v;        // Mains voltage in volts
    uint32_t timestamp_us;
} ws_data_packet_t;

// WebSocket batch structure for sending multiple samples at once
#define WS_BATCH_SIZE 100
typedef struct {
    ws_data_packet_t samples[WS_BATCH_SIZE];
    size_t count;
} ws_batch_packet_t;


extern httpd_handle_t server;

// WebSocket oscilloscope variables
extern volatile uint32_t ws_buffer_index;   // Current position in WebSocket buffer
extern volatile uint32_t ws_decimation_counter; // Counter for decimation
extern QueueHandle_t ws_data_queue;             // Queue for WebSocket data transmission
extern bool ws_client_connected;        // WebSocket client connection status

// WebSocket batch variables
extern ws_batch_packet_t current_batch;
extern size_t batch_index;


esp_err_t root_get_handler(httpd_req_t *req);
esp_err_t api_stats_get_handler(httpd_req_t *req);
esp_err_t api_wifi_get_handler(httpd_req_t *req);
esp_err_t api_wifi_post_handler(httpd_req_t *req);
esp_err_t ws_handler(httpd_req_t *req);
void ws_data_task(void *pvParameters);
httpd_handle_t start_webserver(void);
