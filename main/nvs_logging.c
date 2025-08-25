#include "nvs_logging.h"
#include "adc.h"
#include "rtc_time.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include <math.h>

static const char *TAG = "NVS_LOGGING";

// Global state
static const esp_partition_t *log_partition = NULL;
static uint32_t current_write_offset = 0;
static uint32_t total_entries_written = 0;
static bool logging_initialized = false;
static bool logging_active = false;
static SemaphoreHandle_t logging_mutex = NULL;

// Configuration and averaging state
static log_config_t current_config = {
    .log_interval_seconds = LOG_FREQ_DEFAULT,
    .auto_averaging = true
};
static uint64_t last_log_time_us = 0;
static uint32_t stats_count = 0;
static periodic_statistics_t accumulated_stats = {0};
static bool have_accumulated_data = false;

/*---------------------------------------------------------------
        Private Functions
---------------------------------------------------------------*/

// Calculate CRC32 for log entry data (excluding magic and crc fields)
static uint32_t calculate_entry_crc(const log_entry_t *entry)
{
    const uint8_t *data = (const uint8_t *)&entry->timestamp_us;
    size_t data_size = sizeof(log_entry_t) - sizeof(entry->magic) - sizeof(entry->crc32);
    return esp_crc32_le(0, data, data_size);
}

// Find the next write location by scanning for erased area
static esp_err_t find_write_location(void)
{
    esp_err_t ret;
    uint32_t offset = 0;
    log_entry_t temp_entry;
    
    ESP_LOGI(TAG, "Scanning partition for write location...");
    
    // Scan through the partition to find the first erased entry
    while (offset <= log_partition->size - LOG_ENTRY_SIZE) {
        ret = esp_partition_read(log_partition, offset, &temp_entry, sizeof(log_entry_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read partition at offset %lu", offset);
            return ret;
        }
        
        // Check if this location is erased (all 0xFF)
        if (temp_entry.magic == 0xFFFFFFFF) {
            ESP_LOGI(TAG, "Found write location at offset %lu", offset);
            current_write_offset = offset;
            return ESP_OK;
        }
        
        // Check if this is a valid entry
        if (temp_entry.magic == LOG_MAGIC_NUMBER) {
            uint32_t calculated_crc = calculate_entry_crc(&temp_entry);
            if (calculated_crc == temp_entry.crc32) {
                total_entries_written++;
            }
        }
        
        offset += LOG_ENTRY_SIZE;
    }
    
    // If we get here, the partition is full - start from beginning (circular buffer)
    ESP_LOGW(TAG, "Partition full, starting from beginning (circular buffer mode)");
    current_write_offset = 0;
    
    // Erase the first block to ensure we have space
    ret = esp_partition_erase_range(log_partition, 0, LOG_BLOCK_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase first block");
        return ret;
    }
    
    return ESP_OK;
}

// Ensure we have an erased block ahead of our write position
static esp_err_t ensure_erased_ahead(void)
{
    uint32_t next_block_offset = ((current_write_offset / LOG_BLOCK_SIZE) + 1) * LOG_BLOCK_SIZE;
    
    // If we're at the end of partition, wrap around
    if (next_block_offset >= log_partition->size) {
        next_block_offset = 0;
    }
    
    // Check if the next block needs erasing
    log_entry_t temp_entry;
    esp_err_t ret = esp_partition_read(log_partition, next_block_offset, &temp_entry, sizeof(log_entry_t));
    if (ret != ESP_OK) {
        return ret;
    }
    
    // If the first entry in the next block isn't erased, erase the block
    if (temp_entry.magic != 0xFFFFFFFF) {
        ESP_LOGI(TAG, "Erasing block at offset %lu", next_block_offset);
        ret = esp_partition_erase_range(log_partition, next_block_offset, LOG_BLOCK_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase block at offset %lu", next_block_offset);
            return ret;
        }
    }
    
    return ESP_OK;
}

// Load configuration from NVS
static esp_err_t load_config_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("log_config", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No existing config found, using defaults");
        return ESP_OK; // Use defaults
    }
    
    size_t required_size = sizeof(log_config_t);
    err = nvs_get_blob(nvs_handle, "config", &current_config, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load config from NVS: %s", esp_err_to_name(err));
        // Reset to defaults on error
        current_config.log_interval_seconds = LOG_FREQ_DEFAULT;
        current_config.auto_averaging = true;
    } else {
        ESP_LOGI(TAG, "Loaded config: interval=%lus, averaging=%s", 
                 current_config.log_interval_seconds,
                 current_config.auto_averaging ? "enabled" : "disabled");
    }
    
    nvs_close(nvs_handle);
    return ESP_OK;
}

// Save configuration to NVS
static esp_err_t save_config_to_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("log_config", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for config save: %s", esp_err_to_name(err));
        return err;
    }
    
    err = nvs_set_blob(nvs_handle, "config", &current_config, sizeof(log_config_t));
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
    }
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save config to NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saved config: interval=%lus, averaging=%s", 
                 current_config.log_interval_seconds,
                 current_config.auto_averaging ? "enabled" : "disabled");
    }
    
    nvs_close(nvs_handle);
    return err;
}

// Accumulate statistics for averaging
static void accumulate_statistics(const periodic_statistics_t *stats)
{
    if (!have_accumulated_data) {
        // First sample - initialize
        accumulated_stats = *stats;
        stats_count = 1;
        have_accumulated_data = true;
    } else {
        // Accumulate values for averaging
        accumulated_stats.ac_rms_voltage_scaled += stats->ac_rms_voltage_scaled;
        accumulated_stats.peak_to_peak_scaled += stats->peak_to_peak_scaled;
        accumulated_stats.frequency_hz += stats->frequency_hz;
        
        // For min/max values, take the extremes
        accumulated_stats.min_frequency_hz = fminf(accumulated_stats.min_frequency_hz, stats->min_frequency_hz);
        accumulated_stats.max_frequency_hz = fmaxf(accumulated_stats.max_frequency_hz, stats->max_frequency_hz);
        accumulated_stats.min_voltage_scaled = fminf(accumulated_stats.min_voltage_scaled, stats->min_voltage_scaled);
        accumulated_stats.max_voltage_scaled = fmaxf(accumulated_stats.max_voltage_scaled, stats->max_voltage_scaled);
        
        stats_count++;
    }
}

// Calculate averaged statistics
static periodic_statistics_t get_averaged_statistics(void)
{
    periodic_statistics_t averaged = accumulated_stats;
    
    if (stats_count > 1) {
        averaged.ac_rms_voltage_scaled /= stats_count;
        averaged.peak_to_peak_scaled /= stats_count;
        averaged.frequency_hz /= stats_count;
        // Note: min/max values are already the extremes, no averaging needed
    }
    
    return averaged;
}

// Reset accumulation state
static void reset_accumulation(void)
{
    memset(&accumulated_stats, 0, sizeof(periodic_statistics_t));
    stats_count = 0;
    have_accumulated_data = false;
}

// Write a log entry to flash
static esp_err_t write_log_entry(const log_entry_t *entry)
{
    if (!log_partition || !entry) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if we need to wrap around to the beginning of the partition
    if (current_write_offset + LOG_ENTRY_SIZE > log_partition->size) {
        current_write_offset = 0;
        
        // Erase the first block when we wrap around
        esp_err_t ret = esp_partition_erase_range(log_partition, 0, LOG_BLOCK_SIZE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase first block on wrap-around");
            return ret;
        }
    }
    
    // Ensure we have an erased block ahead of our current position
    esp_err_t ret = ensure_erased_ahead();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Write the entry to flash
    ret = esp_partition_write(log_partition, current_write_offset, entry, sizeof(log_entry_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write log entry at offset %lu", current_write_offset);
        return ret;
    }
    
    // Update tracking variables
    current_write_offset += LOG_ENTRY_SIZE;
    total_entries_written++;
    
    return ESP_OK;
}

/*---------------------------------------------------------------
        Public API Implementation
---------------------------------------------------------------*/

esp_err_t nvs_logging_init(void)
{
    if (logging_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing NVS logging system...");
    
    // Create mutex for thread safety
    logging_mutex = xSemaphoreCreateMutex();
    if (logging_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create logging mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Find the logging partition
    log_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, LOG_PARTITION_LABEL);
    if (log_partition == NULL) {
        ESP_LOGE(TAG, "Logging partition '%s' not found!", LOG_PARTITION_LABEL);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "Found logging partition: size=%lu bytes, address=0x%lx", 
             log_partition->size, log_partition->address);
    
    // Find the current write location
    esp_err_t ret = find_write_location();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to find write location");
        return ret;
    }
    
    // Ensure we have an erased block ahead
    ret = ensure_erased_ahead();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to ensure erased block ahead");
        return ret;
    }
    
    // Load configuration from NVS
    load_config_from_nvs();
    
    logging_initialized = true;
    ESP_LOGI(TAG, "NVS logging initialized. Current offset: %lu, Total entries: %lu", 
             current_write_offset, total_entries_written);
    
    return ESP_OK;
}

esp_err_t nvs_logging_start(void)
{
    if (!logging_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(logging_mutex, portMAX_DELAY);
    logging_active = true;
    xSemaphoreGive(logging_mutex);
    
    // Subscribe to statistics callbacks
    int result = adc_subscribe_statistics(nvs_logging_statistics_callback);
    if (result < 0) {
        ESP_LOGE(TAG, "Failed to subscribe to statistics callbacks");
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "NVS logging started");
    return ESP_OK;
}

esp_err_t nvs_logging_stop(void)
{
    if (!logging_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    xSemaphoreTake(logging_mutex, portMAX_DELAY);
    logging_active = false;
    xSemaphoreGive(logging_mutex);
    
    // Unsubscribe from statistics callbacks
    adc_unsubscribe_statistics(nvs_logging_statistics_callback);
    
    ESP_LOGI(TAG, "NVS logging stopped");
    return ESP_OK;
}

esp_err_t nvs_logging_get_status(log_status_t *status)
{
    if (!logging_initialized || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(logging_mutex, portMAX_DELAY);
    status->total_entries = total_entries_written;
    status->current_offset = current_write_offset;
    status->partition_size = log_partition ? log_partition->size : 0;
    status->entries_written = total_entries_written; // For now, same as total
    status->logging_active = logging_active;
    xSemaphoreGive(logging_mutex);
    
    return ESP_OK;
}

esp_err_t nvs_logging_read_entries(uint32_t start_offset, uint32_t count, log_entry_t *entries, uint32_t *entries_read)
{
    if (!logging_initialized || entries == NULL || entries_read == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *entries_read = 0;
    
    // Calculate starting position - read backwards from current write position
    uint32_t read_position;
    if (current_write_offset == 0) {
        // If we're at the beginning, start from the end (circular buffer)
        read_position = log_partition->size - LOG_ENTRY_SIZE;
    } else {
        read_position = current_write_offset - LOG_ENTRY_SIZE;
    }
    
    // Skip entries based on start_offset (for pagination)
    uint32_t skipped = 0;
    while (skipped < start_offset && skipped < total_entries_written) {
        log_entry_t temp_entry;
        esp_err_t ret = esp_partition_read(log_partition, read_position, &temp_entry, sizeof(log_entry_t));
        if (ret != ESP_OK) {
            return ret;
        }
        
        // Only skip valid entries
        if (temp_entry.magic == LOG_MAGIC_NUMBER) {
            uint32_t calculated_crc = calculate_entry_crc(&temp_entry);
            if (calculated_crc == temp_entry.crc32) {
                skipped++;
            }
        }
        
        // Move to previous entry
        if (read_position == 0) {
            read_position = log_partition->size - LOG_ENTRY_SIZE;
        } else {
            read_position -= LOG_ENTRY_SIZE;
        }
    }
    
    // Read valid entries
    uint32_t attempts = 0;
    uint32_t max_attempts = log_partition->size / LOG_ENTRY_SIZE;
    
    while (*entries_read < count && attempts < max_attempts) {
        log_entry_t temp_entry;
        esp_err_t ret = esp_partition_read(log_partition, read_position, &temp_entry, sizeof(log_entry_t));
        if (ret != ESP_OK) {
            return ret;
        }
        
        attempts++;
        
        // Validate the entry
        if (temp_entry.magic == LOG_MAGIC_NUMBER) {
            uint32_t calculated_crc = calculate_entry_crc(&temp_entry);
            if (calculated_crc == temp_entry.crc32) {
                // Copy valid entry to output array
                entries[*entries_read] = temp_entry;
                (*entries_read)++;
            }
        } else if (temp_entry.magic == 0xFFFFFFFF) {
            // Hit erased area - we might be in the gap, continue looking
            // (don't break immediately as we might have wrapped around)
        }
        
        // Move to previous entry (backwards)
        if (read_position == 0) {
            read_position = log_partition->size - LOG_ENTRY_SIZE;
        } else {
            read_position -= LOG_ENTRY_SIZE;
        }
    }
    
    ESP_LOGI(TAG, "Read %lu valid entries out of %lu attempts", *entries_read, attempts);
    return ESP_OK;
}

esp_err_t nvs_logging_erase_all(void)
{
    if (!logging_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGW(TAG, "Erasing all log data...");
    
    xSemaphoreTake(logging_mutex, portMAX_DELAY);
    
    esp_err_t ret = esp_partition_erase_range(log_partition, 0, log_partition->size);
    if (ret == ESP_OK) {
        current_write_offset = 0;
        total_entries_written = 0;
        ESP_LOGI(TAG, "All log data erased");
    } else {
        ESP_LOGE(TAG, "Failed to erase log data");
    }
    
    xSemaphoreGive(logging_mutex);
    return ret;
}

void nvs_logging_statistics_callback(const periodic_statistics_t *stats)
{
    if (!logging_initialized || !logging_active || stats == NULL) {
        return;
    }
    
    // Take mutex with timeout to avoid blocking in callback
    if (xSemaphoreTake(logging_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex in statistics callback");
        return;
    }
    
    uint64_t current_time_us = esp_timer_get_time();
    uint64_t time_since_last_log_us = current_time_us - last_log_time_us;
    uint64_t log_interval_us = current_config.log_interval_seconds * 1000000ULL;
    
    if (current_config.auto_averaging) {
        // Accumulate statistics for averaging
        accumulate_statistics(stats);
        
        // Check if it's time to log
        if (last_log_time_us == 0 || time_since_last_log_us >= log_interval_us) {
            // Get averaged statistics
            periodic_statistics_t averaged_stats = get_averaged_statistics();
            
            // Prepare log entry with averaged data
            log_entry_t entry = {
                .magic = LOG_MAGIC_NUMBER,
                .timestamp_us = current_time_us,
                .timestamp_unix = rtc_get_time(),
                .boot_counter = get_boot_counter(),
                .ac_rms_voltage_scaled = averaged_stats.ac_rms_voltage_scaled,
                .peak_to_peak_scaled = averaged_stats.peak_to_peak_scaled,
                .frequency_hz = averaged_stats.frequency_hz,
                .min_frequency_hz = averaged_stats.min_frequency_hz,
                .max_frequency_hz = averaged_stats.max_frequency_hz,
                .min_voltage_scaled = averaged_stats.min_voltage_scaled,
                .max_voltage_scaled = averaged_stats.max_voltage_scaled,
            };
            
            // Calculate CRC
            entry.crc32 = calculate_entry_crc(&entry);
            
            // Write the entry
            esp_err_t ret = write_log_entry(&entry);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Logged averaged entry #%lu (from %lu samples), freq=%.2fHz, voltage=%.1fV", 
                         total_entries_written, stats_count, 
                         averaged_stats.frequency_hz, averaged_stats.ac_rms_voltage_scaled);
            }
            
            // Reset accumulation and update timing
            reset_accumulation();
            last_log_time_us = current_time_us;
        }
    } else {
        // No averaging - log every interval
        if (last_log_time_us == 0 || time_since_last_log_us >= log_interval_us) {
            // Prepare log entry with current data
            log_entry_t entry = {
                .magic = LOG_MAGIC_NUMBER,
                .timestamp_us = current_time_us,
                .timestamp_unix = rtc_get_time(),
                .boot_counter = get_boot_counter(),
                .ac_rms_voltage_scaled = stats->ac_rms_voltage_scaled,
                .peak_to_peak_scaled = stats->peak_to_peak_scaled,
                .frequency_hz = stats->frequency_hz,
                .min_frequency_hz = stats->min_frequency_hz,
                .max_frequency_hz = stats->max_frequency_hz,
                .min_voltage_scaled = stats->min_voltage_scaled,
                .max_voltage_scaled = stats->max_voltage_scaled,
            };
            
            // Calculate CRC
            entry.crc32 = calculate_entry_crc(&entry);
            
            // Write the entry
            esp_err_t ret = write_log_entry(&entry);
            if (ret == ESP_OK && total_entries_written % 10 == 0) {
                ESP_LOGI(TAG, "Logged entry #%lu, freq=%.1fHz, voltage=%.1fV", 
                         total_entries_written, stats->frequency_hz, stats->ac_rms_voltage_scaled);
            }
            
            last_log_time_us = current_time_us;
        }
    }
    
    xSemaphoreGive(logging_mutex);
}

/*---------------------------------------------------------------
        Configuration API Functions
---------------------------------------------------------------*/

esp_err_t nvs_logging_set_config(const log_config_t *config)
{
    if (!logging_initialized || config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Validate configuration parameters
    if (config->log_interval_seconds < LOG_FREQ_MIN_SECONDS || 
        config->log_interval_seconds > LOG_FREQ_MAX_SECONDS) {
        ESP_LOGE(TAG, "Invalid log interval: %lu (must be between %d and %d seconds)", 
                 config->log_interval_seconds, LOG_FREQ_MIN_SECONDS, LOG_FREQ_MAX_SECONDS);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex to ensure thread safety
    if (xSemaphoreTake(logging_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for config update");
        return ESP_ERR_TIMEOUT;
    }
    
    // Update configuration
    current_config = *config;
    
    // Reset accumulation state when configuration changes
    reset_accumulation();
    last_log_time_us = 0; // Force immediate log on next callback
    
    xSemaphoreGive(logging_mutex);
    
    // Save to NVS
    esp_err_t ret = save_config_to_nvs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save configuration to NVS: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Configuration updated: interval=%lus, averaging=%s", 
             config->log_interval_seconds,
             config->auto_averaging ? "enabled" : "disabled");
    
    return ESP_OK;
}

esp_err_t nvs_logging_get_config(log_config_t *config)
{
    if (!logging_initialized || config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Take mutex to ensure thread safety
    if (xSemaphoreTake(logging_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for config read");
        return ESP_ERR_TIMEOUT;
    }
    
    *config = current_config;
    
    xSemaphoreGive(logging_mutex);
    return ESP_OK;
}

/*---------------------------------------------------------------
        Timeframe Reading Functions
---------------------------------------------------------------*/

// Calculate Unix time for entries without RTC using boot counter offset
static time_t calculate_unix_time_from_boot(uint64_t timestamp_us, int boot_counter, 
                                           time_t reference_unix_time, uint64_t reference_timestamp_us, 
                                           int reference_boot_counter)
{
    // If boot counters match, we can calculate offset within the same boot cycle
    if (boot_counter == reference_boot_counter) {
        // Calculate the time difference in seconds
        int64_t time_diff_us = (int64_t)timestamp_us - (int64_t)reference_timestamp_us;
        return reference_unix_time + (time_diff_us / 1000000);
    }
    
    // Different boot cycles - we can't reliably calculate this without more context
    return 0;
}

esp_err_t nvs_logging_read_entries_by_timeframe(time_t start_time, time_t end_time, 
                                               nvs_logging_entry_callback_t callback, 
                                               void *user_data)
{
    if (!logging_initialized || callback == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (start_time > end_time) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Take mutex to ensure thread safety
    if (xSemaphoreTake(logging_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex for timeframe reading");
        return ESP_ERR_TIMEOUT;
    }
    
    const uint32_t CHUNK_SIZE = 50; // Process 50 entries at a time
    uint32_t entries_found = 0;
    esp_err_t ret = ESP_OK;
    
    // Calculate starting position - search from current write position backwards
    uint32_t current_offset = current_write_offset;
    if (current_offset == 0) {
        current_offset = log_partition->size; // Start from end if at beginning
    }
    
    // Variables for boot counter offset calculation
    time_t reference_unix_time = 0;
    uint64_t reference_timestamp_us = 0;
    int reference_boot_counter = -1;
    bool have_reference = false;
    
    ESP_LOGI(TAG, "Searching timeframe %lld to %lld, starting from offset %lu", 
             start_time, end_time, current_offset);
    
    // Allocate buffer for chunk processing
    log_entry_t *chunk_buffer = malloc(CHUNK_SIZE * sizeof(log_entry_t));
    if (!chunk_buffer) {
        xSemaphoreGive(logging_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    uint32_t search_offset = current_offset;
    bool found_entries_in_range = false;
    
    // Search backwards through the partition
    while (search_offset > 0 && ret == ESP_OK) {
        // Calculate chunk start position
        uint32_t chunk_start;
        uint32_t entries_to_read;
        
        if (search_offset >= CHUNK_SIZE * sizeof(log_entry_t)) {
            chunk_start = search_offset - (CHUNK_SIZE * sizeof(log_entry_t));
            entries_to_read = CHUNK_SIZE;
        } else {
            chunk_start = 0;
            entries_to_read = search_offset / sizeof(log_entry_t);
        }
        
        if (entries_to_read == 0) break;
        
        // Read chunk from flash
        ret = esp_partition_read(log_partition, chunk_start, chunk_buffer, 
                               entries_to_read * sizeof(log_entry_t));
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read chunk at offset %lu: %s", 
                     chunk_start, esp_err_to_name(ret));
            break;
        }
        
        // Process entries in reverse order (newest first)
        for (int i = entries_to_read - 1; i >= 0 && ret == ESP_OK; i--) {
            log_entry_t *entry = &chunk_buffer[i];
            
            // Validate entry
            if (entry->magic != LOG_MAGIC_NUMBER || entry->magic == 0xFFFFFFFF) {
                continue; // Skip invalid entries
            }
            
            // Validate CRC
            uint32_t calculated_crc = calculate_entry_crc(entry);
            if (calculated_crc != entry->crc32) {
                continue; // Skip corrupted entries
            }
            
            // Calculate effective Unix time for this entry
            time_t effective_unix_time = entry->timestamp_unix;
            
            // If no RTC time set, try to calculate from boot counter offset
            if (effective_unix_time == LOG_VALID_TIMESTAMP && have_reference) {
                effective_unix_time = calculate_unix_time_from_boot(
                    entry->timestamp_us, entry->boot_counter,
                    reference_unix_time, reference_timestamp_us, reference_boot_counter
                );
            }
            
            // Update reference time if this entry has valid RTC time
            if (entry->timestamp_unix > LOG_VALID_TIMESTAMP) {
                reference_unix_time = entry->timestamp_unix;
                reference_timestamp_us = entry->timestamp_us;
                reference_boot_counter = entry->boot_counter;
                have_reference = true;
            }
            
            // Check if entry is in our time range
            if (effective_unix_time >= start_time && effective_unix_time <= end_time && effective_unix_time > LOG_VALID_TIMESTAMP) {
                found_entries_in_range = true;
                
                // Create a copy with the effective timestamp
                log_entry_t effective_entry = *entry;
                effective_entry.timestamp_unix = effective_unix_time;
                
                // Call the callback with this entry
                ret = callback(&effective_entry, user_data);
                if (ret != ESP_OK) {
                    break; // Stop if callback indicates error
                }
                
                entries_found++;
                
                // Yield every 10 entries to prevent watchdog timeout
                if (entries_found % 10 == 0) {
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
            }
            
            // If we've gone past our start time (remember we're going backwards), stop searching
            if (effective_unix_time < start_time && found_entries_in_range && effective_unix_time > LOG_VALID_TIMESTAMP) {
                ESP_LOGI(TAG, "Reached start of time range, stopping search");
                goto search_complete;
            }
        }
        
        // Move to next chunk
        search_offset = chunk_start;
        
        // Yield between chunks to prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
search_complete:
    free(chunk_buffer);
    xSemaphoreGive(logging_mutex);
    
    ESP_LOGI(TAG, "Timeframe search complete: found %lu entries", entries_found);
    return ret;
}
