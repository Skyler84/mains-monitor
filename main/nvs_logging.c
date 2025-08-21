#include "nvs_logging.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "NVS_LOGGING";

// Global state
static const esp_partition_t *log_partition = NULL;
static uint32_t current_write_offset = 0;
static uint32_t total_entries_written = 0;
static bool logging_initialized = false;
static bool logging_active = false;
static SemaphoreHandle_t logging_mutex = NULL;

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

void nvs_logging_statistics_callback(const adc_statistics_t *stats)
{
    if (!logging_initialized || !logging_active || stats == NULL) {
        return;
    }
    
    // Take mutex with timeout to avoid blocking in callback
    if (xSemaphoreTake(logging_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex in statistics callback");
        return;
    }
    
    // Prepare log entry
    log_entry_t entry = {
        .magic = LOG_MAGIC_NUMBER,
        .timestamp_us = esp_timer_get_time(),
        .mean_voltage_mv = stats->mean_voltage_mv,
        .rms_voltage_mv = stats->rms_voltage_mv,
        .ac_rms_voltage_mv = stats->ac_rms_voltage_mv,
        .std_dev_voltage_mv = stats->std_dev_voltage_mv,
        .min_voltage_mv = stats->min_voltage_mv,
        .max_voltage_mv = stats->max_voltage_mv,
        .peak_to_peak_mv = stats->peak_to_peak_mv,
        .ac_rms_voltage_scaled = stats->ac_rms_voltage_scaled,
        .peak_to_peak_scaled = stats->peak_to_peak_scaled,
        .frequency_hz = stats->frequency_hz,
        .zero_crossings = stats->zero_crossings,
    };
    
    // Calculate CRC
    entry.crc32 = calculate_entry_crc(&entry);
    
    // Check if we need to wrap around
    if (current_write_offset + LOG_ENTRY_SIZE > log_partition->size) {
        current_write_offset = 0;
        ESP_LOGI(TAG, "Wrapping to beginning of partition");
    }
    
    // Write the entry
    esp_err_t ret = esp_partition_write(log_partition, current_write_offset, &entry, sizeof(log_entry_t));
    if (ret == ESP_OK) {
        current_write_offset += LOG_ENTRY_SIZE;
        total_entries_written++;
        
        // Log every 10th entry to verify it's working
        if (total_entries_written % 10 == 0) {
            ESP_LOGI(TAG, "Logged entry #%lu at offset %lu, freq=%.1fHz, voltage=%.1fV", 
                     total_entries_written, current_write_offset - LOG_ENTRY_SIZE, 
                     stats->frequency_hz, stats->ac_rms_voltage_scaled);
        }
        
        // Check if we need to erase the next block
        if ((current_write_offset % LOG_BLOCK_SIZE) == 0) {
            ensure_erased_ahead();
        }
    } else {
        ESP_LOGE(TAG, "Failed to write log entry at offset %lu: %s", 
                 current_write_offset, esp_err_to_name(ret));
    }
    
    xSemaphoreGive(logging_mutex);
}
