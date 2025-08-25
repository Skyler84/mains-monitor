#include "adc.h"
#include "led_status.h"
#include "rtc_time.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ADC";

// Callback subscription arrays
static adc_raw_callback_t raw_callbacks[MAX_RAW_CALLBACKS] = {NULL};
static adc_statistics_callback_t statistics_callbacks[MAX_STATISTICS_CALLBACKS] = {NULL};
static SemaphoreHandle_t callback_mutex;

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

// Synchronization
static SemaphoreHandle_t buffer_mutex;
static SemaphoreHandle_t processing_semaphore;

// Timer handle
static esp_timer_handle_t adc_timer;

// Latest statistics for external access
static periodic_statistics_t latest_stats = {0};

// Function prototypes
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static void adc_timer_callback(void* arg);
static void adc_processing_task(void *pvParameters);
static void calculate_statistics(const float *voltage_buffer, periodic_statistics_t *stats);
static void update_system_status(const periodic_statistics_t *stats);
static float IRAM_ATTR apply_moving_average_filter(float new_value);
static float IRAM_ATTR apply_exponential_filter(float new_value);
static void notify_raw_subscribers(float voltage_mv, uint32_t sample_index);
static void notify_statistics_subscribers(const periodic_statistics_t *stats);

/*---------------------------------------------------------------
        Public API Implementation
---------------------------------------------------------------*/

esp_err_t adc_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // Create synchronization primitives
    buffer_mutex = xSemaphoreCreateMutex();
    processing_semaphore = xSemaphoreCreateBinary();
    callback_mutex = xSemaphoreCreateMutex();
    
    if (buffer_mutex == NULL || processing_semaphore == NULL || callback_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create synchronization primitives");
        return ESP_ERR_NO_MEM;
    }

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ret = adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        return ret;
    }

    //-------------ADC1 Calibration Init---------------//
    adc_calibrated = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_handle);

    ESP_LOGI(TAG, "ADC initialization complete");
    ESP_LOGI(TAG, "Filter configuration: Moving Average=%d samples, Exponential Alpha=%.2f", FILTER_SIZE, FILTER_ALPHA);

    // Create processing task
    BaseType_t task_ret = xTaskCreate(adc_processing_task, "adc_processing", 4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create ADC processing task");
        return ESP_ERR_NO_MEM;
    }

    // Create timer for ADC sampling
    esp_timer_create_args_t timer_args = {
        .callback = &adc_timer_callback,
        .arg = NULL,
        .name = "adc_timer"
    };
    ret = esp_timer_create(&timer_args, &adc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ADC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC subsystem initialized successfully");
    return ESP_OK;
}

esp_err_t adc_start_sampling(void)
{
    esp_err_t ret = esp_timer_start_periodic(adc_timer, SAMPLE_PERIOD_US);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start ADC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC sampling started at %d Hz. Statistics will be calculated every %d samples.", 
             SAMPLE_RATE_HZ, BUFFER_SIZE);
    return ESP_OK;
}

esp_err_t adc_stop_sampling(void)
{
    esp_err_t ret = esp_timer_stop(adc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop ADC timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC sampling stopped");
    return ESP_OK;
}

void adc_cleanup(void)
{
    // Stop and delete timer
    if (adc_timer) {
        esp_timer_stop(adc_timer);
        esp_timer_delete(adc_timer);
        adc_timer = NULL;
    }
    
    // Clean up ADC
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    
    if (adc_calibrated && adc1_cali_handle) {
        example_adc_calibration_deinit(adc1_cali_handle);
        adc1_cali_handle = NULL;
        adc_calibrated = false;
    }
    
    // Clean up synchronization primitives
    if (buffer_mutex) {
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
    }
    
    if (processing_semaphore) {
        vSemaphoreDelete(processing_semaphore);
        processing_semaphore = NULL;
    }
    
    if (callback_mutex) {
        vSemaphoreDelete(callback_mutex);
        callback_mutex = NULL;
    }
    
    ESP_LOGI(TAG, "ADC subsystem cleaned up");
}

const periodic_statistics_t* adc_get_latest_stats(void)
{
    return &latest_stats;
}

/*---------------------------------------------------------------
        Callback Subscription System
---------------------------------------------------------------*/
int adc_subscribe_raw_values(adc_raw_callback_t callback)
{
    if (callback == NULL) return -1;
    
    xSemaphoreTake(callback_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_RAW_CALLBACKS; i++) {
        if (raw_callbacks[i] == NULL) {
            raw_callbacks[i] = callback;
            xSemaphoreGive(callback_mutex);
            ESP_LOGI(TAG, "Raw value callback subscribed at index %d", i);
            return i;
        }
    }
    xSemaphoreGive(callback_mutex);
    ESP_LOGW(TAG, "Failed to subscribe raw value callback - all slots full");
    return -1;
}

int adc_subscribe_statistics(adc_statistics_callback_t callback)
{
    if (callback == NULL) return -1;
    
    xSemaphoreTake(callback_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_STATISTICS_CALLBACKS; i++) {
        if (statistics_callbacks[i] == NULL) {
            statistics_callbacks[i] = callback;
            xSemaphoreGive(callback_mutex);
            ESP_LOGI(TAG, "Statistics callback subscribed at index %d", i);
            return i;
        }
    }
    xSemaphoreGive(callback_mutex);
    ESP_LOGW(TAG, "Failed to subscribe statistics callback - all slots full");
    return -1;
}

int adc_unsubscribe_raw_values(adc_raw_callback_t callback)
{
    if (callback == NULL) return -1;
    
    xSemaphoreTake(callback_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_RAW_CALLBACKS; i++) {
        if (raw_callbacks[i] == callback) {
            raw_callbacks[i] = NULL;
            xSemaphoreGive(callback_mutex);
            ESP_LOGI(TAG, "Raw value callback unsubscribed from index %d", i);
            return i;
        }
    }
    xSemaphoreGive(callback_mutex);
    ESP_LOGW(TAG, "Raw value callback not found for unsubscription");
    return -1;
}

int adc_unsubscribe_statistics(adc_statistics_callback_t callback)
{
    if (callback == NULL) return -1;
    
    xSemaphoreTake(callback_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_STATISTICS_CALLBACKS; i++) {
        if (statistics_callbacks[i] == callback) {
            statistics_callbacks[i] = NULL;
            xSemaphoreGive(callback_mutex);
            ESP_LOGI(TAG, "Statistics callback unsubscribed from index %d", i);
            return i;
        }
    }
    xSemaphoreGive(callback_mutex);
    ESP_LOGW(TAG, "Statistics callback not found for unsubscription");
    return -1;
}

/*---------------------------------------------------------------
        Internal Functions
---------------------------------------------------------------*/

static void notify_raw_subscribers(float voltage_mv, uint32_t sample_index)
{
    // ISR-safe version - don't take mutex in ISR, just call callbacks directly
    // Subscribers must be aware they might be called from ISR context
    for (int i = 0; i < MAX_RAW_CALLBACKS; i++) {
        if (raw_callbacks[i] != NULL) {
            raw_callbacks[i](voltage_mv, sample_index);
        }
    }
}

static void notify_statistics_subscribers(const periodic_statistics_t *stats)
{
    xSemaphoreTake(callback_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_STATISTICS_CALLBACKS; i++) {
        if (statistics_callbacks[i] != NULL) {
            statistics_callbacks[i](stats);
        }
    }
    xSemaphoreGive(callback_mutex);
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
        static uint32_t adc_error_count = 0;
        adc_error_count++;
        if (adc_error_count > 100) { // Multiple consecutive failures
            led_status_update(LED_FLAG_ADC_ERROR);
            ESP_LOGE(TAG, "Persistent ADC read failures");
        }
        return; // Skip this sample if read fails
    }
    
    // Convert to voltage immediately if calibration is available
    float voltage_mv = 0.0f;
    if (adc_calibrated && adc1_cali_handle != NULL) {
        int voltage_raw;
        if (adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage_raw) == ESP_OK) {
            voltage_mv = (float)voltage_raw;
        } else {
            static uint32_t cali_error_count = 0;
            cali_error_count++;
            if (cali_error_count > 100) { // Multiple consecutive failures
                led_status_update(LED_FLAG_ADC_ERROR);
                ESP_LOGE(TAG, "Persistent ADC calibration failures");
            }
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
    
    // Notify raw value subscribers (only if callbacks exist to avoid overhead)
    // Note: We notify with the filtered voltage since that's what gets stored
    if (raw_callbacks[0] != NULL) { // Quick check if any subscribers exist
        notify_raw_subscribers(filtered_voltage, buffer_index);
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
    periodic_statistics_t stats;
    
    while (1) {
        // Wait for buffer to be ready
        if (xSemaphoreTake(processing_semaphore, portMAX_DELAY) == pdTRUE) {
            if (buffer_ready_for_processing && processing_voltage_buffer != NULL) {
                
                // Calculate statistics on the completed buffer
                calculate_statistics((const float*)processing_voltage_buffer, &stats);
                
                // Update latest stats for external access
                latest_stats = stats;
                
                // Check system status and update LEDs
                update_system_status(&stats);
                
                // Notify statistics subscribers
                notify_statistics_subscribers(&stats);
                
                buffer_ready_for_processing = false;
            }
        }
    }
}

/*---------------------------------------------------------------
        System Status Monitoring
---------------------------------------------------------------*/
static void update_system_status(const periodic_statistics_t *stats)
{
    uint32_t status_flags = LED_FLAG_NONE;
    
    // Check if RTC is set
    if (!rtc_is_time_set()) {
        status_flags |= LED_FLAG_RTC_NOT_SET;
    }
    
    // Check voltage levels (reasonable mains voltage range: 100V - 260V RMS)
    float voltage_rms = stats->ac_rms_voltage_scaled;
    if (voltage_rms > 260.0f) {
        status_flags |= LED_FLAG_VOLTAGE_HIGH;
    } else if (voltage_rms < 100.0f && voltage_rms > 5.0f) { // Ignore very low readings (no voltage present)
        status_flags |= LED_FLAG_VOLTAGE_LOW;
    }
    
    // Check frequency (reasonable mains frequency range: 45Hz - 65Hz)
    float frequency = stats->frequency_hz;
    if (frequency > 65.0f) {
        status_flags |= LED_FLAG_FREQ_HIGH;
    } else if (frequency < 45.0f && frequency > 5.0f) { // Ignore very low readings (no signal)
        status_flags |= LED_FLAG_FREQ_LOW;
    }
    
    // Update LED status
    led_status_update(status_flags);
    
    // Log warnings if any
    if (status_flags != LED_FLAG_NONE) {
        ESP_LOGW(TAG, "System status flags: 0x%lx", status_flags);
        if (status_flags & LED_FLAG_RTC_NOT_SET) ESP_LOGW(TAG, "  - RTC not set");
        if (status_flags & LED_FLAG_VOLTAGE_HIGH) ESP_LOGW(TAG, "  - Voltage high: %.1fV", voltage_rms);
        if (status_flags & LED_FLAG_VOLTAGE_LOW) ESP_LOGW(TAG, "  - Voltage low: %.1fV", voltage_rms);
        if (status_flags & LED_FLAG_FREQ_HIGH) ESP_LOGW(TAG, "  - Frequency high: %.1fHz", frequency);
        if (status_flags & LED_FLAG_FREQ_LOW) ESP_LOGW(TAG, "  - Frequency low: %.1fHz", frequency);
    }
}

/*---------------------------------------------------------------
        Statistics Calculation - All in Voltage Domain
---------------------------------------------------------------*/
static void calculate_statistics(const float *voltage_buffer, periodic_statistics_t *stats)
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
    uint32_t prev_crossing_sample = 0;  // Track previous crossing for interval measurement
    bool above_mean = (voltage_buffer[0] > stats->mean_voltage_mv);
    bool found_first_crossing = false;

    const uint32_t MIN_CROSSING_INTERVAL = 25;  // Minimum samples between crossings (Schmitt trigger)

    // For min/max frequency tracking
    float min_cycle_freq = INFINITY;
    float max_cycle_freq = -INFINITY;

    for (int i = 1; i < BUFFER_SIZE; i++) {
        bool current_above_mean = (voltage_buffer[i] > stats->mean_voltage_mv);

        // Detect crossing: state changed from above to below or below to above
        if (current_above_mean != above_mean) {
            // Apply Schmitt trigger: only count crossing if enough samples have passed since last crossing
            if (zero_crossings == 0 || (i - last_crossing_sample) >= MIN_CROSSING_INTERVAL) {
                zero_crossings++;
                prev_crossing_sample = last_crossing_sample;
                last_crossing_sample = i;

                // Record first crossing index
                if (!found_first_crossing) {
                    first_crossing_index = i;
                    found_first_crossing = true;
                }

                // Always update last crossing index
                last_crossing_index = i;

                // If we have a previous crossing, compute instantaneous cycle frequency
                if (prev_crossing_sample != 0) {
                    uint32_t half_cycle_samples = last_crossing_sample - prev_crossing_sample;
                    if (half_cycle_samples > 0) {
                        // Full cycle frequency = SAMPLE_RATE_HZ / (2 * half_cycle_samples)
                        float cycle_freq = (float)SAMPLE_RATE_HZ / (2.0f * (float)half_cycle_samples);
                        if (cycle_freq < min_cycle_freq) min_cycle_freq = cycle_freq;
                        if (cycle_freq > max_cycle_freq) max_cycle_freq = cycle_freq;
                    }
                }

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

    // Populate min/max frequency (if we never updated min_cycle_freq it means we didn't have intervals)
    if (min_cycle_freq == INFINITY) {
        stats->min_frequency_hz = stats->frequency_hz;
    } else {
        stats->min_frequency_hz = min_cycle_freq;
    }
    stats->max_frequency_hz = max_cycle_freq > 0.0f ? max_cycle_freq : stats->frequency_hz;

    // Scale to mains voltage
    stats->ac_rms_voltage_scaled = (stats->ac_rms_voltage_mv / 1000.0f) * TOTAL_SCALING;
    stats->peak_to_peak_scaled = (stats->peak_to_peak_mv / 1000.0f) * TOTAL_SCALING;

    // Time period covered by this statistics block
    stats->time_period_s = (float)BUFFER_SIZE / (float)SAMPLE_RATE_HZ;
}

// Accumulate/merge statistics: accum := weighted average of accum and src using their time_period_s
void adc_accumulate_statistics(periodic_statistics_t *accum, const periodic_statistics_t *src)
{
    if (accum == NULL || src == NULL) return;

    // If accum has no time period (uninitialized), simply copy src
    if (accum->time_period_s <= 0.0f) {
        *accum = *src;
        return;
    }

    float t1 = accum->time_period_s;
    float t2 = src->time_period_s > 0.0f ? src->time_period_s : 0.0f;
    float total_t = t1 + t2;

    if (total_t <= 0.0f) return; // nothing to do

    // Weighted averages for linear quantities
    accum->mean_voltage_mv = (accum->mean_voltage_mv * t1 + src->mean_voltage_mv * t2) / total_t;
    accum->rms_voltage_mv = (accum->rms_voltage_mv * t1 + src->rms_voltage_mv * t2) / total_t;
    accum->ac_rms_voltage_mv = (accum->ac_rms_voltage_mv * t1 + src->ac_rms_voltage_mv * t2) / total_t;
    accum->std_dev_voltage_mv = (accum->std_dev_voltage_mv * t1 + src->std_dev_voltage_mv * t2) / total_t;
    accum->ac_rms_voltage_scaled = (accum->ac_rms_voltage_scaled * t1 + src->ac_rms_voltage_scaled * t2) / total_t;
    accum->peak_to_peak_scaled = (accum->peak_to_peak_scaled * t1 + src->peak_to_peak_scaled * t2) / total_t;
    accum->frequency_hz = (accum->frequency_hz * t1 + src->frequency_hz * t2) / total_t;

    // For min/max take the extremes
    accum->min_voltage_mv = fminf(accum->min_voltage_mv, src->min_voltage_mv);
    accum->max_voltage_mv = fmaxf(accum->max_voltage_mv, src->max_voltage_mv);
    accum->min_frequency_hz = fminf(accum->min_frequency_hz, src->min_frequency_hz);
    accum->max_frequency_hz = fmaxf(accum->max_frequency_hz, src->max_frequency_hz);

    // Peak-to-peak (mv) can be recalculated from min/max if desired
    accum->peak_to_peak_mv = accum->max_voltage_mv - accum->min_voltage_mv;

    // Zero crossings are additive over non-overlapping time periods
    accum->zero_crossings += src->zero_crossings;

    // Update time period
    accum->time_period_s = total_t;
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
