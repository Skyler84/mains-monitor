#pragma once

#include <stdint.h>
#include <stddef.h>


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

// ADC Filtering configuration
#define FILTER_SIZE                 5                   // Moving average filter size (5 samples)
#define FILTER_ALPHA                0.1f               // Low-pass filter coefficient (0.1 = heavy filtering)


/*---------------------------------------------------*/
/*                   Typedefs*/
/*---------------------------------------------------*/

// Maximum number of subscribers per callback type
#define MAX_RAW_CALLBACKS       5
#define MAX_STATISTICS_CALLBACKS 5

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

// Callback function types
typedef void (*adc_raw_callback_t)(float voltage_mv, uint32_t sample_index);
typedef void (*adc_statistics_callback_t)(const adc_statistics_t *stats);

/*---------------------------------------------------*/
/*                   Public API*/
/*---------------------------------------------------*/

// Callback subscription functions
int adc_subscribe_raw_values(adc_raw_callback_t callback);
int adc_subscribe_statistics(adc_statistics_callback_t callback);
int adc_unsubscribe_raw_values(adc_raw_callback_t callback);
int adc_unsubscribe_statistics(adc_statistics_callback_t callback);

// Latest statistics for web display
extern adc_statistics_t latest_stats;

// Boot counter access
int get_boot_counter(void);
