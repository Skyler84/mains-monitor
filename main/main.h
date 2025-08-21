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

// WebSocket oscilloscope configuration
#define WS_SAMPLE_RATE_HZ           1000                // WebSocket data rate (1kHz for oscilloscope)
#define WS_BUFFER_SIZE              200                 // WebSocket buffer size (200ms of data at 1kHz)
#define WS_DECIMATION_FACTOR        10                  // Send every 10th sample (10kHz -> 1kHz)

/*---------------------------------------------------*/
/*                   Typedefs*/
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
} adc_statistics_t;

// Latest statistics for web display
extern adc_statistics_t latest_stats;
