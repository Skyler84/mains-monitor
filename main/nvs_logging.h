#pragma once

#include "main.h"
#include "adc.h"
#include "esp_err.h"
#include "esp_partition.h"
#include <time.h>

// Logging configuration
#define LOG_PARTITION_LABEL     "data_log"
#define LOG_ENTRY_SIZE          sizeof(log_entry_t)
#define LOG_BLOCK_SIZE          4096                    // Flash erase block size
#define LOG_ENTRIES_PER_BLOCK   (LOG_BLOCK_SIZE / LOG_ENTRY_SIZE)
#define LOG_BUFFER_BLOCKS       16                      // Number of blocks to keep as buffer
#define LOG_MAGIC_NUMBER        0xADC12345              // Magic number to identify valid entries

// Logging frequency configuration
#define LOG_FREQ_MIN_SECONDS    1                       // Minimum logging interval (1 second)
#define LOG_FREQ_MAX_SECONDS    300                     // Maximum logging interval (5 minutes)
#define LOG_FREQ_DEFAULT        1                       // Default logging interval (1 second)

#define LOG_VALID_TIMESTAMP     1755644400              // Approx. 2025-08-20 (to validate timestamps)

// Logging configuration structure
typedef struct {
    uint32_t log_interval_seconds;  // Logging interval in seconds (1-300)
    bool auto_averaging;            // Whether to average data over the interval
} log_config_t;

// Log entry structure - slimmed down for efficient flash storage
typedef struct {
    uint32_t magic;                 // Magic number for validation
    uint64_t timestamp_us;          // Microseconds since boot (for precise timing)
    time_t timestamp_unix;          // Unix timestamp (seconds since epoch)
    int boot_counter;               // Boot counter for ordering across reboots
    float ac_rms_voltage_scaled;    // AC RMS scaled to mains voltage
    float peak_to_peak_scaled;      // Peak-to-peak scaled to mains voltage
    float frequency_hz;             // Measured frequency from zero crossings
    uint32_t crc32;                 // CRC32 checksum of the data
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

// Configure logging frequency and averaging
esp_err_t nvs_logging_set_config(const log_config_t *config);
esp_err_t nvs_logging_get_config(log_config_t *config);

// Read log entries in reverse chronological order (newest first)
// start_offset: number of recent entries to skip (for pagination)
// count: maximum number of entries to read
esp_err_t nvs_logging_read_entries(uint32_t start_offset, uint32_t count, log_entry_t *entries, uint32_t *entries_read);

// Read log entries by timeframe with callback for chunked processing
// start_time: Unix timestamp for start of range
// end_time: Unix timestamp for end of range
// callback: function called for each entry found in range
// user_data: pointer passed to callback function
typedef esp_err_t (*nvs_logging_entry_callback_t)(const log_entry_t *entry, void *user_data);
esp_err_t nvs_logging_read_entries_by_timeframe(time_t start_time, time_t end_time, 
                                               nvs_logging_entry_callback_t callback, 
                                               void *user_data);

// Erase all log data (factory reset)
esp_err_t nvs_logging_erase_all(void);

// Statistics callback function (internal)
void nvs_logging_statistics_callback(const periodic_statistics_t *stats);
