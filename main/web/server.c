#include "server.h"
#include "wifi.h"
#include "nvs_logging.h"
#include "rtc_time.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

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

static esp_err_t send_entry_http_callback(const log_entry_t *entry, void *user_data);

/*---------------------------------------------------------------
        WebSocket Callback Functions
---------------------------------------------------------------*/
void ws_raw_data_callback(float voltage_mv, uint32_t sample_index)
{
    // WebSocket oscilloscope data collection (decimated)
    ws_decimation_counter++;
    if (ws_decimation_counter >= WS_DECIMATION_FACTOR && ws_client_connected) {
        ws_decimation_counter = 0;
        
        // Remove DC bias from the filtered voltage before scaling
        float ac_voltage_mv = voltage_mv - latest_stats.mean_voltage_mv;
        
        // Scale AC voltage to mains voltage
        float mains_voltage = (ac_voltage_mv / 1000.0f) * TOTAL_SCALING;
        
        // Add sample to current batch
        current_batch.samples[batch_index].voltage_v = mains_voltage;
        current_batch.samples[batch_index].timestamp_us = esp_timer_get_time();
        batch_index++;
        
        // Send batch when full
        if (batch_index >= WS_BATCH_SIZE) {
            current_batch.count = batch_index;
            BaseType_t result = xQueueSend(ws_data_queue, &current_batch, 0); // Don't block in callback
            if (result != pdTRUE) {
                ESP_LOGW(TAG, "WebSocket queue full, dropping data batch");
            }
            batch_index = 0; // Reset batch
        }
    }
}



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
        History API Handlers
---------------------------------------------------------------*/
esp_err_t api_history_status_get_handler(httpd_req_t *req)
{
    log_status_t status;
    esp_err_t ret = nvs_logging_get_status(&status);
    
    if (ret != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to get logging status\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    char json_response[512];
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"status\":\"success\","
        "\"data\":{"
        "\"total_entries\":%lu,"
        "\"current_offset\":%lu,"
        "\"partition_size\":%lu,"
        "\"entries_written\":%lu,"
        "\"logging_active\":%s"
        "}"
        "}",
        status.total_entries,
        status.current_offset,
        status.partition_size,
        status.entries_written,
        status.logging_active ? "true" : "false"
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

esp_err_t api_history_data_get_handler(httpd_req_t *req)
{
    // Parse query parameters
    char query[256];
    size_t query_len = sizeof(query);
    esp_err_t ret = httpd_req_get_url_query_str(req, query, query_len);
    
    time_t start_time = 0;
    time_t end_time = 0;
    bool use_time_range = false;
    
    if (ret == ESP_OK) {
        char param[32];
        
        // Parse start time (Unix timestamp)
        if (httpd_query_key_value(query, "start", param, sizeof(param)) == ESP_OK) {
            start_time = atol(param);
            use_time_range = true;
        }
        
        // Parse end time (Unix timestamp)  
        if (httpd_query_key_value(query, "end", param, sizeof(param)) == ESP_OK) {
            end_time = atol(param);
            use_time_range = true;
        }
    }
    
    // If no time range specified, default to last 24 hours
    if (!use_time_range) {
        end_time = rtc_get_time();
        start_time = end_time - (24 * 60 * 60); // 24 hours ago
    }
    
    // Validate time range
    if (start_time > end_time) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Start time must be before end time\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    ESP_LOGI(TAG, "API: Reading entries from %lld to %lld (Unix timestamps)", start_time, end_time);
    
    // Start JSON response
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    
    char json_header[256];
    snprintf(json_header, sizeof(json_header), 
        "{\"status\":\"success\",\"data\":{\"start_time\":%lld,\"end_time\":%lld,\"entries\":[",
        start_time, end_time);
    httpd_resp_send_chunk(req, json_header, strlen(json_header));
    
    // Set up callback context
    http_response_context_t ctx = {
        .req = req,
        .first_entry = true,
        .entries_sent = 0,
        .result = ESP_OK
    };
    
    // Read and send entries using the nvs_logging API
    ret = nvs_logging_read_entries_by_timeframe(start_time, end_time, send_entry_http_callback, &ctx);
    
    if (ret != ESP_OK || ctx.result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read timeframe entries: nvs_ret=%s, http_ret=%s", 
                 esp_err_to_name(ret), esp_err_to_name(ctx.result));
        // If we failed partway through, try to close the JSON gracefully
        httpd_resp_send_chunk(req, "]}}", 3);
        httpd_resp_send_chunk(req, NULL, 0);
        return (ret != ESP_OK) ? ret : ctx.result;
    }
    
    ESP_LOGI(TAG, "Successfully sent %lu entries for timeframe %lld to %lld", 
             ctx.entries_sent, start_time, end_time);
    
    // Close JSON
    httpd_resp_send_chunk(req, "]}}", 3);
    httpd_resp_send_chunk(req, NULL, 0); // End chunked response
    
    return ESP_OK;
}

esp_err_t api_history_clear_post_handler(httpd_req_t *req)
{
    esp_err_t ret = nvs_logging_erase_all();
    
    if (ret == ESP_OK) {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"status\":\"success\",\"message\":\"History cleared successfully\"}", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to clear history\"}", HTTPD_RESP_USE_STRLEN);
    }
}

esp_err_t api_history_erase_post_handler(httpd_req_t *req)
{
    ESP_LOGW(TAG, "FLASH PARTITION ERASE requested - this will destroy all historical data!");
    
    esp_err_t ret = nvs_logging_erase_all();
    
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "Flash partition erased successfully");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, "{\"status\":\"success\",\"message\":\"Flash partition erased successfully\"}", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGE(TAG, "Failed to erase flash partition: %s", esp_err_to_name(ret));
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to erase flash partition\"}", HTTPD_RESP_USE_STRLEN);
    }
}

/*---------------------------------------------------------------
        RTC Time API Handlers
---------------------------------------------------------------*/
esp_err_t api_time_get_handler(httpd_req_t *req)
{
    time_t current_time = rtc_get_time();
    bool time_set = rtc_is_time_set();
    
    char time_string[64];
    char iso8601_string[32];
    
    rtc_get_time_string(time_string, sizeof(time_string));
    rtc_time_to_iso8601(current_time, iso8601_string, sizeof(iso8601_string));
    
    char json_response[256];
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"status\":\"success\","
        "\"data\":{"
        "\"unix_timestamp\":%lld,"
        "\"iso8601\":\"%s\","
        "\"human_readable\":\"%s\","
        "\"time_set\":%s"
        "}"
        "}",
        current_time,
        iso8601_string,
        time_string,
        time_set ? "true" : "false"
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

esp_err_t api_time_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"No data received\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    buf[ret] = '\0';
    
    // Parse JSON: {"timestamp": 1234567890} or {"iso8601": "2023-01-01T00:00:00Z"}
    char *timestamp_start = strstr(buf, "\"timestamp\":");
    char *iso8601_start = strstr(buf, "\"iso8601\":\"");
    
    time_t new_time = 0;
    esp_err_t parse_result = ESP_ERR_INVALID_ARG;
    
    if (timestamp_start) {
        // Parse Unix timestamp
        timestamp_start += 12; // Skip "timestamp":
        new_time = atol(timestamp_start);
        if (new_time > 1577836800) { // Sanity check: after 2020-01-01
            parse_result = ESP_OK;
        }
    } else if (iso8601_start) {
        // Parse ISO8601 string
        iso8601_start += 11; // Skip "iso8601":"
        char *iso8601_end = strchr(iso8601_start, '"');
        if (iso8601_end) {
            *iso8601_end = '\0';
            parse_result = rtc_iso8601_to_time(iso8601_start, &new_time);
        }
    }
    
    if (parse_result == ESP_OK) {
        esp_err_t set_result = rtc_set_time(new_time);
        if (set_result == ESP_OK) {
            char time_str[64];
            rtc_get_time_string(time_str, sizeof(time_str));
            
            char response[200];
            snprintf(response, sizeof(response),
                "{\"status\":\"success\",\"message\":\"Time set successfully\",\"time\":\"%s\"}",
                time_str);
            
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        } else {
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to set system time\"}", HTTPD_RESP_USE_STRLEN);
        }
    } else {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Invalid time format. Use {\\\"timestamp\\\": 1234567890} or {\\\"iso8601\\\": \\\"2023-01-01T00:00:00Z\\\"}\"}", HTTPD_RESP_USE_STRLEN);
    }
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
    config.max_uri_handlers = 20; // Increase max URI handlers for additional API endpoints
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

        // History status GET handler
        httpd_uri_t api_history_status = {
            .uri       = "/api/history/status",
            .method    = HTTP_GET,
            .handler   = api_history_status_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_history_status);

        // History data GET handler
        httpd_uri_t api_history_data = {
            .uri       = "/api/history/data",
            .method    = HTTP_GET,
            .handler   = api_history_data_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_history_data);

        // History clear POST handler
        httpd_uri_t api_history_clear = {
            .uri       = "/api/history/clear",
            .method    = HTTP_POST,
            .handler   = api_history_clear_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_history_clear);

        // History erase POST handler (destructive flash partition erase)
        httpd_uri_t api_history_erase = {
            .uri       = "/api/history/erase",
            .method    = HTTP_POST,
            .handler   = api_history_erase_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_history_erase);

        // Time GET handler
        httpd_uri_t api_time_get = {
            .uri       = "/api/time",
            .method    = HTTP_GET,
            .handler   = api_time_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_time_get);

        // Time POST handler (for setting time)
        httpd_uri_t api_time_post = {
            .uri       = "/api/time",
            .method    = HTTP_POST,
            .handler   = api_time_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_time_post);

        // WebSocket oscilloscope handler
        httpd_uri_t ws = {
            .uri        = "/ws",
            .method     = HTTP_GET,
            .handler    = ws_handler,
            .user_ctx   = NULL,
            .is_websocket = true
        };
        httpd_register_uri_handler(server, &ws);

        // Logging configuration GET handler
        httpd_uri_t api_logging_config_get = {
            .uri       = "/api/logging/config",
            .method    = HTTP_GET,
            .handler   = api_logging_config_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_logging_config_get);

        // Logging configuration POST handler
        httpd_uri_t api_logging_config_post = {
            .uri       = "/api/logging/config",
            .method    = HTTP_POST,
            .handler   = api_logging_config_post_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &api_logging_config_post);

        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

/*---------------------------------------------------------------
        Timeframe Reading Helper Functions  
---------------------------------------------------------------*/

// Callback function for sending entries via HTTP chunked response
static esp_err_t send_entry_http_callback(const log_entry_t *entry, void *user_data)
{
    http_response_context_t *ctx = (http_response_context_t *)user_data;
    
    // Format entry as JSON
    char entry_json[350];
    char iso8601_time[32] = "null";
    
    if (entry->timestamp_unix > 0) {
        rtc_time_to_iso8601(entry->timestamp_unix, iso8601_time, sizeof(iso8601_time));
    }
    
    snprintf(entry_json, sizeof(entry_json),
        "%s{"
        "\"timestamp_us\":%llu,"
        "\"timestamp_unix\":%lld,"
        "\"timestamp_iso8601\":\"%s\","
        "\"boot_counter\":%d,"
        "\"ac_rms_voltage_scaled\":%.2f,"
        "\"peak_to_peak_scaled\":%.2f,"
        "\"frequency_hz\":%.2f"
        "}",
        ctx->first_entry ? "" : ",",
        entry->timestamp_us,
        entry->timestamp_unix,
        iso8601_time,
        entry->boot_counter,
        entry->ac_rms_voltage_scaled,
        entry->peak_to_peak_scaled,
        entry->frequency_hz
    );
    
    // Send the JSON chunk
    esp_err_t ret = httpd_resp_send_chunk(ctx->req, entry_json, strlen(entry_json));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send HTTP chunk");
        ctx->result = ret;
        return ret;
    }
    
    ctx->first_entry = false;
    ctx->entries_sent++;
    
    return ESP_OK;
}

/*---------------------------------------------------------------
        Logging Configuration API Handlers
---------------------------------------------------------------*/
esp_err_t api_logging_config_get_handler(httpd_req_t *req)
{
    log_config_t config;
    esp_err_t ret = nvs_logging_get_config(&config);
    
    if (ret != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to get logging configuration\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    char json_response[256];
    snprintf(json_response, sizeof(json_response),
        "{"
        "\"status\":\"success\","
        "\"data\":{"
        "\"log_interval_seconds\":%lu,"
        "\"auto_averaging\":%s,"
        "\"min_interval\":%d,"
        "\"max_interval\":%d"
        "}"
        "}",
        config.log_interval_seconds,
        config.auto_averaging ? "true" : "false",
        LOG_FREQ_MIN_SECONDS,
        LOG_FREQ_MAX_SECONDS
    );
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, HTTPD_RESP_USE_STRLEN);
}

esp_err_t api_logging_config_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"No data received\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    buf[ret] = '\0';
    
    // Parse JSON: {"log_interval_seconds": 30, "auto_averaging": true}
    char *interval_start = strstr(buf, "\"log_interval_seconds\":");
    char *averaging_start = strstr(buf, "\"auto_averaging\":");
    
    if (!interval_start) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Missing log_interval_seconds parameter\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    // Parse interval
    interval_start += 23; // Skip "log_interval_seconds":
    ESP_LOGI(TAG, "Parsing log interval from: %s", interval_start);
    uint32_t interval = atoi(interval_start);

    // Validate interval
    if (interval < LOG_FREQ_MIN_SECONDS || interval > LOG_FREQ_MAX_SECONDS) {
        char error_msg[128];
        snprintf(error_msg, sizeof(error_msg), 
            "{\"status\":\"error\",\"message\":\"Invalid interval: %lu (must be %d-%d seconds)\"}",
            interval, LOG_FREQ_MIN_SECONDS, LOG_FREQ_MAX_SECONDS);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, error_msg, HTTPD_RESP_USE_STRLEN);
    }
    
    // Parse averaging (optional, defaults to true)
    bool averaging = true;
    if (averaging_start) {
        averaging_start += 17; // Skip "auto_averaging":
        // Skip whitespace and check for 'f' (false) or 't' (true)
        while (*averaging_start == ' ' || *averaging_start == '\t') averaging_start++;
        if (*averaging_start == 'f') {
            averaging = false;
        }
    }
    
    // Create new configuration
    log_config_t new_config = {
        .log_interval_seconds = interval,
        .auto_averaging = averaging
    };
    
    // Apply configuration
    esp_err_t config_ret = nvs_logging_set_config(&new_config);
    if (config_ret != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"message\":\"Failed to update logging configuration\"}", HTTPD_RESP_USE_STRLEN);
    }
    
    // Send success response
    char response[200];
    snprintf(response, sizeof(response),
        "{\"status\":\"success\",\"message\":\"Logging configuration updated\",\"interval\":%lu,\"averaging\":%s}",
        interval, averaging ? "true" : "false");
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}
