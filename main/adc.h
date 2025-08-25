#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/*---------------------------------------------------*/
/*                   Configuration                   */
/*---------------------------------------------------*/

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

// Maximum number of subscribers per callback type
#define MAX_RAW_CALLBACKS       5
#define MAX_STATISTICS_CALLBACKS 5

/*---------------------------------------------------*/
/*                   Typedefs                        */
/*---------------------------------------------------*/

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
    float min_frequency_hz;         // Minimum frequency measured during the period
    float max_frequency_hz;         // Maximum frequency measured during the period
    float min_voltage_scaled;       // Minimum RMS voltage scaled to mains voltage (tracked across periods)
    float max_voltage_scaled;       // Maximum RMS voltage scaled to mains voltage (tracked across periods)
    float time_period_s;            // Duration (in seconds) that these statistics cover
} periodic_statistics_t;

// Callback function types
typedef void (*adc_raw_callback_t)(float voltage_mv, uint32_t sample_index);
typedef void (*adc_statistics_callback_t)(const periodic_statistics_t *stats);

/*---------------------------------------------------*/
/*                   Public API                      */
/*---------------------------------------------------*/

// ADC initialization and control
esp_err_t adc_init(void);
esp_err_t adc_start_sampling(void);
esp_err_t adc_stop_sampling(void);
void adc_cleanup(void);

// Callback subscription functions
int adc_subscribe_raw_values(adc_raw_callback_t callback);
int adc_subscribe_statistics(adc_statistics_callback_t callback);
int adc_unsubscribe_raw_values(adc_raw_callback_t callback);
int adc_unsubscribe_statistics(adc_statistics_callback_t callback);

// Latest statistics access
const periodic_statistics_t* adc_get_latest_stats(void);

// Merge/accumulate statistics: accum := weighted average of accum and src using their time_period_s
void adc_accumulate_statistics(periodic_statistics_t *accum, const periodic_statistics_t *src);
