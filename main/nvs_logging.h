#pragma once

#include "main.h"
#include "esp_err.h"
#include "esp_partition.h"

// Logging configuration
#define LOG_PARTITION_LABEL     "data_log"
#define LOG_ENTRY_SIZE          sizeof(log_entry_t)
#define LOG_BLOCK_SIZE          4096                    // Flash erase block size
#define LOG_ENTRIES_PER_BLOCK   (LOG_BLOCK_SIZE / LOG_ENTRY_SIZE)
#define LOG_BUFFER_BLOCKS       16                      // Number of blocks to keep as buffer
#define LOG_MAGIC_NUMBER        0xADC12345              // Magic number to identify valid entries

// Log entry structure - matches adc_statistics_t but with timestamp and magic
typedef struct {
    uint32_t magic;                 // Magic number for validation
    uint64_t timestamp_us;          // Timestamp in microseconds since boot
    float mean_voltage_mv;          // DC bias voltage
    float rms_voltage_mv;          // Total RMS voltage
    float ac_rms_voltage_mv;       // AC RMS voltage (DC bias removed)
    float std_dev_voltage_mv;      // Standard deviation in mV
    float min_voltage_mv;          // Minimum voltage
    float max_voltage_mv;          // Maximum voltage
    float peak_to_peak_mv;         // Peak-to-peak voltage
    float ac_rms_voltage_scaled;   // AC RMS scaled to mains voltage
    float peak_to_peak_scaled;     // Peak-to-peak scaled to mains voltage
    float frequency_hz;            // Measured frequency from zero crossings
    uint32_t zero_crossings;       // Number of zero crossings detected
    uint32_t crc32;                // CRC32 checksum of the data
} __attribute__((packed)) log_entry_t;

// Logging status structure
typedef struct {
    uint32_t total_entries;         // Total number of log entries written
    uint32_t current_offset;        // Current write offset in partition
    uint32_t partition_size;        // Total partition size
    uint32_t entries_written;       // Number of entries written this session
    bool logging_active;            // Whether logging is currently active
} log_status_t;

/*---------------------------------------------------------------
        Public API
---------------------------------------------------------------*/

// Initialize the logging system
esp_err_t nvs_logging_init(void);

// Start/stop logging
esp_err_t nvs_logging_start(void);
esp_err_t nvs_logging_stop(void);

// Get logging status
esp_err_t nvs_logging_get_status(log_status_t *status);

// Read log entries in reverse chronological order (newest first)
// start_offset: number of recent entries to skip (for pagination)
// count: maximum number of entries to read
esp_err_t nvs_logging_read_entries(uint32_t start_offset, uint32_t count, log_entry_t *entries, uint32_t *entries_read);

// Erase all log data (factory reset)
esp_err_t nvs_logging_erase_all(void);

// Statistics callback function (internal)
void nvs_logging_statistics_callback(const adc_statistics_t *stats);
