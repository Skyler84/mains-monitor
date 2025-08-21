#include "server.h"
#include "wifi.h"

#include <stddef.h>

#include "esp_log.h"

static const char *TAG = "WEB_SERVER";

// Web server handle
httpd_handle_t server = NULL;


// WebSocket oscilloscope variables
volatile uint32_t ws_buffer_index = 0;   // Current position in WebSocket buffer
volatile uint32_t ws_decimation_counter = 0; // Counter for decimation
QueueHandle_t ws_data_queue;             // Queue for WebSocket data transmission
bool ws_client_connected = false;        // WebSocket client connection status

// WebSocket batch variables
ws_batch_packet_t current_batch = {0};
size_t batch_index = 0;



// External references to embedded files
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t favicon_start[] asm("_binary_electric_plug_png_start");
extern const uint8_t favicon_end[]   asm("_binary_electric_plug_png_end");

/*---------------------------------------------------------------
        Web Server Handlers
---------------------------------------------------------------*/
esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/png");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=604800"); // Cache for 1 week
    httpd_resp_send(req, (const char*)favicon_start, favicon_end - favicon_start);
    return ESP_OK;
}

esp_err_t api_stats_get_handler(httpd_req_t *req)
{
    char json_response[512];
    
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"dc_bias\":%.1f,"
        "\"ac_rms\":%.1f,"
        "\"peak_peak\":%.2f,"
        "\"frequency\":%.3f,"
        "\"zero_crossings\":%lu,"
        "\"mains_rms\":%.2f,"
        "\"mains_peak\":%.2f"
        "}",
        latest_stats.mean_voltage_mv,
        latest_stats.ac_rms_voltage_mv,
        latest_stats.peak_to_peak_mv,
        latest_stats.frequency_hz,
        latest_stats.zero_crossings,
        latest_stats.ac_rms_voltage_scaled,
        latest_stats.peak_to_peak_scaled
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

esp_err_t api_wifi_get_handler(httpd_req_t *req)
{
    char json_response[256];
    
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"ssid\":\"%s\","
        "\"password\":\"%s\","
        "\"mode\":%d"
        "}",
        current_wifi_config.ssid,
        current_wifi_config.password,
        current_wifi_config.mode
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

esp_err_t api_wifi_post_handler(httpd_req_t *req)
{
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    // Parse JSON (simple parsing for our specific format)
    char *ssid_start = strstr(content, "\"ssid\":\"");
    char *password_start = strstr(content, "\"password\":\"");
    char *mode_start = strstr(content, "\"mode\":");
    
    if (ssid_start && password_start && mode_start) {
        // Extract SSID
        ssid_start += 8; // Skip "ssid":"
        char *ssid_end = strchr(ssid_start, '"');
        if (ssid_end) {
            size_t ssid_len = ssid_end - ssid_start;
            if (ssid_len < sizeof(current_wifi_config.ssid)) {
                strncpy(current_wifi_config.ssid, ssid_start, ssid_len);
                current_wifi_config.ssid[ssid_len] = '\0';
            }
        }
        
        // Extract password
        password_start += 12; // Skip "password":"
        char *password_end = strchr(password_start, '"');
        if (password_end) {
            size_t password_len = password_end - password_start;
            if (password_len < sizeof(current_wifi_config.password)) {
                strncpy(current_wifi_config.password, password_start, password_len);
                current_wifi_config.password[password_len] = '\0';
            }
        }
        
        // Extract mode
        mode_start += 7; // Skip "mode":
        current_wifi_config.mode = atoi(mode_start);
        
        // Save configuration
        wifi_save_config();
        
        // Send success response
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"status\":\"success\",\"message\":\"WiFi settings updated. Restart device to apply.\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Invalid JSON format\"}", HTTPD_RESP_USE_STRLEN);
}

/*---------------------------------------------------------------
        WebSocket Handler
---------------------------------------------------------------*/
esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake initiated");
        ws_client_connected = true;
        return ESP_OK;
    }
    
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    
    // Receive WebSocket frame
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        ws_client_connected = false;
        return ret;
    }
    
    if (ws_pkt.len) {
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            ws_client_connected = false;
            return ret;
        }
    }
    
    // Handle different frame types
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        ESP_LOGI(TAG, "Received packet with message: %s", ws_pkt.payload);
    } else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "WebSocket connection closed");
        ws_client_connected = false;
        
        // Flush any remaining batch data
        if (batch_index > 0) {
            current_batch.count = batch_index;
            xQueueSend(ws_data_queue, &current_batch, 0); // Non-blocking send
            batch_index = 0;
        }
    }
    
    if (buf) {
        free(buf);
    }
    return ESP_OK;
}

/*---------------------------------------------------------------
        WebSocket Data Transmission Task
---------------------------------------------------------------*/
void ws_data_task(void *pvParameters)
{
    ws_batch_packet_t batch;
    char json_buffer[8192]; // Larger buffer for batch data
    httpd_ws_frame_t ws_pkt;
    
    while (1) {
        // Wait for batch data from queue
        if (xQueueReceive(ws_data_queue, &batch, portMAX_DELAY) == pdTRUE) {
            if (ws_client_connected && server != NULL) {
                // Format batch data as JSON array
                int offset = snprintf(json_buffer, sizeof(json_buffer), "{\"samples\":[");
                
                for (size_t i = 0; i < batch.count && offset < sizeof(json_buffer) - 50; i++) {
                    if (i > 0) {
                        offset += snprintf(json_buffer + offset, sizeof(json_buffer) - offset, ",");
                    }
                    offset += snprintf(json_buffer + offset, sizeof(json_buffer) - offset,
                        "{\"voltage\":%.2f,\"timestamp\":%lu}",
                        batch.samples[i].voltage_v,
                        (unsigned long)(batch.samples[i].timestamp_us / 1000) // Convert to milliseconds
                    );
                }
                
                offset += snprintf(json_buffer + offset, sizeof(json_buffer) - offset, 
                    "],\"count\":%zu}", batch.count);
                
                // Prepare WebSocket frame
                memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
                ws_pkt.payload = (uint8_t*)json_buffer;
                ws_pkt.len = strlen(json_buffer);
                ws_pkt.type = HTTPD_WS_TYPE_TEXT;
                
                // Send to all WebSocket clients
                size_t clients = 10;
                int client_fds[10];
                esp_err_t ret = httpd_get_client_list(server, &clients, client_fds);
                
                if (ret == ESP_OK) {
                    for (size_t i = 0; i < clients; ++i) {
                        int client_info = httpd_ws_get_fd_info(server, client_fds[i]);
                        if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
                            httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
                        }
                    }
                }
            }
        }
    }
}

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Root page handler
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t favicon = {
            .uri       = "/favicon.ico",
            .method    = HTTP_GET,
            .handler   = favicon_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &favicon);

        // API stats handler
        httpd_uri_t api_stats = {
            .uri       = "/api/stats",
            .method    = HTTP_GET,
            .handler   = api_stats_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_stats);

        // WiFi config GET handler
        httpd_uri_t api_wifi_get = {
            .uri       = "/api/wifi",
            .method    = HTTP_GET,
            .handler   = api_wifi_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_wifi_get);

        // WiFi config POST handler
        httpd_uri_t api_wifi_post = {
            .uri       = "/api/wifi",
            .method    = HTTP_POST,
            .handler   = api_wifi_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_wifi_post);

        // WebSocket oscilloscope handler
        httpd_uri_t ws = {
            .uri        = "/ws",
            .method     = HTTP_GET,
            .handler    = ws_handler,
            .user_ctx   = NULL,
            .is_websocket = true
        };
        httpd_register_uri_handler(server, &ws);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}
