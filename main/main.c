#include "main.h"
#include "wifi.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

static const char *TAG = "ADC_MONITOR";

// Dual buffer system - now stores voltage values in mV
static float voltage_buffer_a[BUFFER_SIZE];
static float voltage_buffer_b[BUFFER_SIZE];
static volatile float *current_voltage_buffer = voltage_buffer_a;
static volatile float *processing_voltage_buffer = NULL;
static volatile uint32_t buffer_index = 0;
static volatile bool buffer_ready_for_processing = false;

// ADC filtering variables
static float filter_buffer[FILTER_SIZE] = {0};  // Circular buffer for moving average
static uint32_t filter_index = 0;               // Current position in filter buffer
static float filter_sum = 0.0f;                 // Running sum for moving average
static float last_filtered_value = 0.0f;        // For exponential smoothing

// ADC handles
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibrated = false;

// Web server handle
static httpd_handle_t server = NULL;

// WebSocket oscilloscope variables
static volatile uint32_t ws_buffer_index = 0;   // Current position in WebSocket buffer
static volatile uint32_t ws_decimation_counter = 0; // Counter for decimation
static QueueHandle_t ws_data_queue;             // Queue for WebSocket data transmission
static bool ws_client_connected = false;        // WebSocket client connection status

// WebSocket batch variables
static ws_batch_packet_t current_batch = {0};
static size_t batch_index = 0;

// Synchronization
static SemaphoreHandle_t buffer_mutex;
static SemaphoreHandle_t processing_semaphore;


// Latest statistics for web display
static adc_statistics_t latest_stats = {0};

// External references to embedded files
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// Function prototypes
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static void adc_timer_callback(void* arg);
static void adc_processing_task(void *pvParameters);
static void calculate_statistics(const float *voltage_buffer, adc_statistics_t *stats);
static esp_err_t root_get_handler(httpd_req_t *req);
static esp_err_t api_stats_get_handler(httpd_req_t *req);
static esp_err_t api_wifi_get_handler(httpd_req_t *req);
static esp_err_t api_wifi_post_handler(httpd_req_t *req);
static esp_err_t ws_handler(httpd_req_t *req);
static void ws_data_task(void *pvParameters);
static httpd_handle_t start_webserver(void);

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize networking
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Load WiFi configuration from NVS
    wifi_load_config();
    
    // Create synchronization primitives
    buffer_mutex = xSemaphoreCreateMutex();
    processing_semaphore = xSemaphoreCreateBinary();
    ws_data_queue = xQueueCreate(10, sizeof(ws_batch_packet_t)); // Queue for 10 WebSocket batch packets
    
    if (buffer_mutex == NULL || processing_semaphore == NULL || ws_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        return;
    }

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_calibrated = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_handle);

    ESP_LOGI(TAG, "ADC initialization complete. Starting continuous sampling at %d Hz with filtering...", SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Filter configuration: Moving Average=%d samples, Exponential Alpha=%.2f", FILTER_SIZE, FILTER_ALPHA);

    // Create processing task
    xTaskCreate(adc_processing_task, "adc_processing", 4096, NULL, 5, NULL);
    
    // Create WebSocket data transmission task
    xTaskCreate(ws_data_task, "ws_data", 4096*3, NULL, 4, NULL);

    // Create and start timer for ADC sampling
    esp_timer_create_args_t timer_args = {
        .callback = &adc_timer_callback,
        .arg = NULL,
        .name = "adc_timer"
    };
    esp_timer_handle_t adc_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &adc_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(adc_timer, SAMPLE_PERIOD_US));

    ESP_LOGI(TAG, "Continuous ADC sampling started. Statistics will be calculated every %d samples.", BUFFER_SIZE);

    // Initialize WiFi and start web server
    wifi_apply_config();
    server = start_webserver();
    if (server) {
        if (current_wifi_config.mode == 0) {
            ESP_LOGI(TAG, "Web server started in AP mode. Connect to WiFi '%s' and browse to http://192.168.4.1", current_wifi_config.ssid);
        } else {
            ESP_LOGI(TAG, "Web server started in Station mode. Attempting to connect to WiFi '%s'...", current_wifi_config.ssid);
            ESP_LOGI(TAG, "The IP address will be displayed once connected. Then browse to that IP address.");
        }
    }

    // Main task just monitors the system
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        // ESP_LOGI(TAG, "System running... Buffer index: %lu", buffer_index);
    }

    // Cleanup (never reached in this implementation)
    esp_timer_stop(adc_timer);
    esp_timer_delete(adc_timer);
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (adc_calibrated) {
        example_adc_calibration_deinit(adc1_cali_handle);
    }
    vSemaphoreDelete(buffer_mutex);
    vSemaphoreDelete(processing_semaphore);
}

/*---------------------------------------------------------------
        ADC Filtering Functions
---------------------------------------------------------------*/
static float IRAM_ATTR apply_moving_average_filter(float new_value)
{
    // Remove oldest value from sum
    filter_sum -= filter_buffer[filter_index];
    
    // Add new value to buffer and sum
    filter_buffer[filter_index] = new_value;
    filter_sum += new_value;
    
    // Move to next position (circular buffer)
    filter_index = (filter_index + 1) % FILTER_SIZE;
    
    // Return average
    return filter_sum / FILTER_SIZE;
}

static float IRAM_ATTR apply_exponential_filter(float new_value)
{
    // Simple exponential smoothing: output = alpha * input + (1-alpha) * previous_output
    last_filtered_value = FILTER_ALPHA * new_value + (1.0f - FILTER_ALPHA) * last_filtered_value;
    return last_filtered_value;
}

/*---------------------------------------------------------------
        ADC Timer Callback - Called at 10kHz
---------------------------------------------------------------*/
static void IRAM_ATTR adc_timer_callback(void* arg)
{
    int adc_raw = 0;
    
    // Read ADC value
    esp_err_t ret = adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw);
    if (ret != ESP_OK) {
        return; // Skip this sample if read fails
    }
    
    // Convert to voltage immediately if calibration is available
    float voltage_mv = 0.0f;
    if (adc_calibrated && adc1_cali_handle != NULL) {
        int voltage_raw;
        if (adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_raw) == ESP_OK) {
            voltage_mv = (float)voltage_raw;
        } else {
            return; // Skip this sample if conversion fails
        }
    } else {
        // Fallback: approximate conversion (assuming 3.3V reference, 12-bit ADC)
        voltage_mv = (float)adc_raw * 3300.0f / 4095.0f;
    }
    
    // Apply filtering to reduce noise
    // First apply moving average to remove high-frequency noise
    float filtered_voltage = apply_moving_average_filter(voltage_mv);
    
    // Then apply exponential smoothing for additional noise reduction
    filtered_voltage = apply_exponential_filter(filtered_voltage);
    
    // Store filtered voltage sample in current buffer
    ((float*)current_voltage_buffer)[buffer_index] = filtered_voltage;
    
    // WebSocket oscilloscope data collection (decimated)
    ws_decimation_counter++;
    if (ws_decimation_counter >= WS_DECIMATION_FACTOR && ws_client_connected) {
        ws_decimation_counter = 0;
        
        // Remove DC bias from the filtered voltage before scaling
        float ac_voltage_mv = filtered_voltage - latest_stats.mean_voltage_mv;
        
        // Scale AC voltage to mains voltage
        float mains_voltage = (ac_voltage_mv / 1000.0f) * TOTAL_SCALING;
        
        // Add sample to current batch
        current_batch.samples[batch_index].voltage_v = mains_voltage;
        current_batch.samples[batch_index].timestamp_us = esp_timer_get_time();
        batch_index++;
        
        // Send batch when full
        if (batch_index >= WS_BATCH_SIZE) {
            current_batch.count = batch_index;
            xQueueSendFromISR(ws_data_queue, &current_batch, NULL);
            batch_index = 0; // Reset batch
        }
    }
    
    buffer_index++;
    
    // Check if buffer is full
    if (buffer_index >= BUFFER_SIZE) {
        // Buffer is full, switch buffers
        if (current_voltage_buffer == voltage_buffer_a) {
            processing_voltage_buffer = voltage_buffer_a;
            current_voltage_buffer = voltage_buffer_b;
        } else {
            processing_voltage_buffer = voltage_buffer_b;
            current_voltage_buffer = voltage_buffer_a;
        }
        
        buffer_index = 0;
        buffer_ready_for_processing = true;
        
        // Signal processing task
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(processing_semaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*---------------------------------------------------------------
        ADC Processing Task
---------------------------------------------------------------*/
static void adc_processing_task(void *pvParameters)
{
    adc_statistics_t stats;
    
    while (1) {
        // Wait for buffer to be ready
        if (xSemaphoreTake(processing_semaphore, portMAX_DELAY) == pdTRUE) {
            if (buffer_ready_for_processing && processing_voltage_buffer != NULL) {
                
                // Calculate statistics on the completed buffer
                calculate_statistics((const float*)processing_voltage_buffer, &stats);
                
                // Update latest stats for web display
                latest_stats = stats;
                
                // Print statistics - all in voltage domain
                // ESP_LOGI(TAG, "=== Voltage Statistics (10000 samples) ===");
                // ESP_LOGI(TAG, "DC Bias: %.1f mV", stats.mean_voltage_mv);
                // ESP_LOGI(TAG, "Total RMS: %.1f mV, AC RMS: %.1f mV", 
                //          stats.rms_voltage_mv, stats.ac_rms_voltage_mv);
                // ESP_LOGI(TAG, "Min: %.1f mV, Max: %.1f mV, Peak-Peak: %.1f mV", 
                //          stats.min_voltage_mv, stats.max_voltage_mv, stats.peak_to_peak_mv);
                // ESP_LOGI(TAG, "Standard Deviation: %.1f mV", stats.std_dev_voltage_mv);
                // ESP_LOGI(TAG, "--- Frequency Analysis ---");
                // ESP_LOGI(TAG, "Zero Crossings: %lu, Frequency: %.2f Hz", 
                //          stats.zero_crossings, stats.frequency_hz);
                // ESP_LOGI(TAG, "--- Scaled Mains Voltage ---");
                // ESP_LOGI(TAG, "AC RMS: %.1f V, Peak-Peak: %.1f V", 
                //          stats.ac_rms_voltage_scaled, stats.peak_to_peak_scaled);
                // ESP_LOGI(TAG, "=====================================");
                
                buffer_ready_for_processing = false;
            }
        }
    }
}

/*---------------------------------------------------------------
        Statistics Calculation - All in Voltage Domain
---------------------------------------------------------------*/
static void calculate_statistics(const float *voltage_buffer, adc_statistics_t *stats)
{
    double sum = 0.0;
    double sum_squares = 0.0;
    float min_voltage = voltage_buffer[0];
    float max_voltage = voltage_buffer[0];
    
    // First pass: calculate basic statistics
    for (int i = 0; i < BUFFER_SIZE; i++) {
        float voltage = voltage_buffer[i];
        sum += voltage;
        sum_squares += voltage * voltage;
        
        if (voltage < min_voltage) min_voltage = voltage;
        if (voltage > max_voltage) max_voltage = voltage;
    }
    
    // Calculate mean voltage (DC bias)
    stats->mean_voltage_mv = (float)(sum / BUFFER_SIZE);
    
    // Calculate total RMS voltage
    stats->rms_voltage_mv = sqrtf((float)(sum_squares / BUFFER_SIZE));
    
    // Calculate AC RMS voltage (RMS with DC bias removed)
    // AC RMS = sqrt(total_rms² - dc_mean²)
    float dc_squared = stats->mean_voltage_mv * stats->mean_voltage_mv;
    stats->ac_rms_voltage_mv = sqrtf((float)(sum_squares / BUFFER_SIZE) - dc_squared);
    
    // Calculate standard deviation (same as AC RMS for DC-biased AC signals)
    stats->std_dev_voltage_mv = stats->ac_rms_voltage_mv;
    
    // Set min/max voltages
    stats->min_voltage_mv = min_voltage;
    stats->max_voltage_mv = max_voltage;
    
    // Calculate peak-to-peak voltage
    stats->peak_to_peak_mv = max_voltage - min_voltage;
    
    // Second pass: count zero crossings for frequency measurement with Schmitt trigger
    uint32_t zero_crossings = 0;
    uint32_t first_crossing_index = 0;
    uint32_t last_crossing_index = 0;
    uint32_t last_crossing_sample = 0;  // Track last crossing position for Schmitt trigger
    bool above_mean = (voltage_buffer[0] > stats->mean_voltage_mv);
    bool found_first_crossing = false;
    
    const uint32_t MIN_CROSSING_INTERVAL = 25;  // Minimum samples between crossings (Schmitt trigger)
    
    for (int i = 1; i < BUFFER_SIZE; i++) {
        bool current_above_mean = (voltage_buffer[i] > stats->mean_voltage_mv);
        
        // Detect crossing: state changed from above to below or below to above
        if (current_above_mean != above_mean) {
            // Apply Schmitt trigger: only count crossing if enough samples have passed since last crossing
            if (zero_crossings == 0 || (i - last_crossing_sample) >= MIN_CROSSING_INTERVAL) {
                zero_crossings++;
                last_crossing_sample = i;
                
                // Record first crossing index
                if (!found_first_crossing) {
                    first_crossing_index = i;
                    found_first_crossing = true;
                }
                
                // Always update last crossing index
                last_crossing_index = i;
                above_mean = current_above_mean;
            }
            // If crossing is too soon after last one, ignore it but don't update above_mean
            // This prevents the state from changing until a valid crossing occurs
        }
    }
    
    stats->zero_crossings = zero_crossings;
    
    // Calculate frequency from zero crossings using actual time between crossings
    if (zero_crossings >= 2 && found_first_crossing) {
        // Calculate actual time between first and last crossing
        uint32_t crossing_span_samples = last_crossing_index - first_crossing_index;
        float crossing_span_seconds = (float)crossing_span_samples / (float)SAMPLE_RATE_HZ;
        
        // Number of complete cycles in the crossing span
        // Each cycle has 2 zero crossings, so (zero_crossings - 1) gives us the crossings between first and last
        float cycles_in_span = (float)(zero_crossings - 1) / 2.0f;
        
        if (cycles_in_span > 0.0f && crossing_span_seconds > 0.0f) {
            stats->frequency_hz = cycles_in_span / crossing_span_seconds;
        } else {
            stats->frequency_hz = 0.0f;
        }
    } else {
        stats->frequency_hz = 0.0f; // Not enough crossings to determine frequency
    }
    
    // Scale to mains voltage
    stats->ac_rms_voltage_scaled = (stats->ac_rms_voltage_mv / 1000.0f) * TOTAL_SCALING;
    stats->peak_to_peak_scaled = (stats->peak_to_peak_mv / 1000.0f) * TOTAL_SCALING;
}

/*---------------------------------------------------------------
        Web Server Handlers
---------------------------------------------------------------*/
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char*)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t api_stats_get_handler(httpd_req_t *req)
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

static esp_err_t api_wifi_get_handler(httpd_req_t *req)
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

static esp_err_t api_wifi_post_handler(httpd_req_t *req)
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
static esp_err_t ws_handler(httpd_req_t *req)
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
static void ws_data_task(void *pvParameters)
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

static httpd_handle_t start_webserver(void)
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

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}