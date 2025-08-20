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

static const char *TAG = "ADC_MONITOR";

#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

// Buffer configuration
#define BUFFER_SIZE                 10000
#define SAMPLE_RATE_HZ              10000
#define SAMPLE_PERIOD_US            (1000000 / SAMPLE_RATE_HZ)  // 100 microseconds

// Voltage scaling configuration
#define ISOLATION_RATIO             (242.5f / 8.4f)    // 242.5V to 8.4V isolation transformer
#define STEPDOWN_RATIO              11.0f              // 11:1 voltage divider
#define TOTAL_SCALING               (ISOLATION_RATIO * STEPDOWN_RATIO)  // Total scaling factor

// Dual buffer system - now stores voltage values in mV
static float voltage_buffer_a[BUFFER_SIZE];
static float voltage_buffer_b[BUFFER_SIZE];
static volatile float *current_voltage_buffer = voltage_buffer_a;
static volatile float *processing_voltage_buffer = NULL;
static volatile uint32_t buffer_index = 0;
static volatile bool buffer_ready_for_processing = false;

// ADC handles
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool adc_calibrated = false;

// Synchronization
static SemaphoreHandle_t buffer_mutex;
static SemaphoreHandle_t processing_semaphore;

// Statistics structure - all values in voltage domain
typedef struct {
    float mean_voltage_mv;           // DC bias voltage
    float rms_voltage_mv;           // Total RMS voltage
    float ac_rms_voltage_mv;        // AC RMS voltage (DC bias removed)
    float std_dev_voltage_mv;       // Standard deviation in mV
    float min_voltage_mv;           // Minimum voltage
    float max_voltage_mv;           // Maximum voltage
    float peak_to_peak_mv;          // Peak-to-peak voltage
    float ac_rms_voltage_scaled;    // AC RMS scaled to mains voltage
    float peak_to_peak_scaled;      // Peak-to-peak scaled to mains voltage
    float frequency_hz;             // Measured frequency from zero crossings
    uint32_t zero_crossings;        // Number of zero crossings detected
} adc_statistics_t;

// Function prototypes
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static void adc_timer_callback(void* arg);
static void adc_processing_task(void *pvParameters);
static void calculate_statistics(const float *voltage_buffer, adc_statistics_t *stats);

void app_main(void)
{
    // Create synchronization primitives
    buffer_mutex = xSemaphoreCreateMutex();
    processing_semaphore = xSemaphoreCreateBinary();
    
    if (buffer_mutex == NULL || processing_semaphore == NULL) {
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

    ESP_LOGI(TAG, "ADC initialization complete. Starting continuous sampling at %d Hz...", SAMPLE_RATE_HZ);

    // Create processing task
    xTaskCreate(adc_processing_task, "adc_processing", 4096, NULL, 5, NULL);

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
    
    // Store voltage sample in current buffer
    ((float*)current_voltage_buffer)[buffer_index] = voltage_mv;
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
                
                // Print statistics - all in voltage domain
                ESP_LOGI(TAG, "=== Voltage Statistics (10000 samples) ===");
                ESP_LOGI(TAG, "DC Bias: %.1f mV", stats.mean_voltage_mv);
                ESP_LOGI(TAG, "Total RMS: %.1f mV, AC RMS: %.1f mV", 
                         stats.rms_voltage_mv, stats.ac_rms_voltage_mv);
                ESP_LOGI(TAG, "Min: %.1f mV, Max: %.1f mV, Peak-Peak: %.1f mV", 
                         stats.min_voltage_mv, stats.max_voltage_mv, stats.peak_to_peak_mv);
                ESP_LOGI(TAG, "Standard Deviation: %.1f mV", stats.std_dev_voltage_mv);
                ESP_LOGI(TAG, "--- Frequency Analysis ---");
                ESP_LOGI(TAG, "Zero Crossings: %lu, Frequency: %.2f Hz", 
                         stats.zero_crossings, stats.frequency_hz);
                ESP_LOGI(TAG, "--- Scaled Mains Voltage ---");
                ESP_LOGI(TAG, "AC RMS: %.1f V, Peak-Peak: %.1f V", 
                         stats.ac_rms_voltage_scaled, stats.peak_to_peak_scaled);
                ESP_LOGI(TAG, "=====================================");
                
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