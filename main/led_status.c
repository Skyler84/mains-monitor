#include "led_status.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

static const char *TAG = "LED_STATUS";

// LED strip handle for Neopixel
static led_strip_handle_t led_strip = NULL;

// Current status tracking
static led_status_t current_status = LED_STATUS_OK;
static uint32_t current_flags = LED_FLAG_NONE;

// Blink timer
static TimerHandle_t blink_timer = NULL;
static uint32_t blink_count_remaining = 0;
static led_status_t blink_status = LED_STATUS_OK;

/*---------------------------------------------------------------
        Private Functions
---------------------------------------------------------------*/

static void set_neopixel_color(uint8_t red, uint8_t green, uint8_t blue)
{
    if (led_strip) {
        led_strip_set_pixel(led_strip, 0, red, green, blue);
        led_strip_refresh(led_strip);
    }
}

static void apply_status_to_leds(led_status_t status)
{
    switch (status) {
        case LED_STATUS_OK:
            set_neopixel_color(0, 50, 0);  // Green (dim)
            break;
        case LED_STATUS_WARNING:
            set_neopixel_color(50, 25, 0); // Orange
            break;
        case LED_STATUS_ERROR:
            set_neopixel_color(50, 0, 0);  // Red
            break;
        default:
            set_neopixel_color(0, 0, 0);   // Off
            break;
    }
}

static void blink_timer_callback(TimerHandle_t xTimer)
{
    static bool blink_state = false;
    
    if (blink_count_remaining > 0) {
        if (blink_state) {
            // Turn on with blink status color
            apply_status_to_leds(blink_status);
            blink_count_remaining--;
        } else {
            // Turn off
            set_neopixel_color(0, 0, 0);
        }
        blink_state = !blink_state;
    } else {
        // Blinking finished, return to normal status
        xTimerStop(blink_timer, 0);
        apply_status_to_leds(current_status);
    }
}

/*---------------------------------------------------------------
        Public API Implementation
---------------------------------------------------------------*/

esp_err_t led_status_init(void)
{
    ESP_LOGI(TAG, "Initializing LED status system...");
    
    // Configure user LED GPIO
    gpio_config_t user_led_config = {
        .pin_bit_mask = (1ULL << USER_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&user_led_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure user LED GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize user LED to off
    gpio_set_level(USER_LED_GPIO, 0);
    
    // Configure Neopixel LED strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = NEOPIXEL_GPIO,
        .max_leds = 1, // Only one LED on the board
    };
    
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    
    ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create blink timer
    blink_timer = xTimerCreate(
        "blink_timer",
        pdMS_TO_TICKS(250), // Default 250ms period
        pdTRUE,             // Auto-reload
        NULL,
        blink_timer_callback
    );
    
    if (blink_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create blink timer");
        return ESP_ERR_NO_MEM;
    }
    
    // Set initial status to OK (green)
    led_status_set(LED_STATUS_OK);
    
    ESP_LOGI(TAG, "LED status system initialized");
    return ESP_OK;
}

esp_err_t led_status_update(uint32_t flags)
{
    current_flags = flags;
    
    // Determine status based on flags
    led_status_t new_status = LED_STATUS_OK;
    
    // Check for critical errors first
    if (flags & (LED_FLAG_ADC_ERROR | LED_FLAG_CRITICAL_ERROR)) {
        new_status = LED_STATUS_ERROR;
    }
    // Check for warnings
    else if (flags & (LED_FLAG_RTC_NOT_SET | LED_FLAG_VOLTAGE_HIGH | LED_FLAG_VOLTAGE_LOW | 
                     LED_FLAG_FREQ_HIGH | LED_FLAG_FREQ_LOW | LED_FLAG_WIFI_ERROR)) {
        new_status = LED_STATUS_WARNING;
    }
    
    return led_status_set(new_status);
}

esp_err_t led_status_set(led_status_t status)
{
    current_status = status;
    
    // Don't change LED if we're currently blinking
    if (xTimerIsTimerActive(blink_timer)) {
        return ESP_OK;
    }
    
    apply_status_to_leds(status);
    
    const char* status_str = (status == LED_STATUS_OK) ? "OK" : 
                            (status == LED_STATUS_WARNING) ? "WARNING" : "ERROR";
    ESP_LOGI(TAG, "LED status set to %s", status_str);
    
    return ESP_OK;
}

esp_err_t led_user_set(bool on)
{
    gpio_set_level(USER_LED_GPIO, on ? 1 : 0);
    ESP_LOGD(TAG, "User LED %s", on ? "ON" : "OFF");
    return ESP_OK;
}

led_status_t led_status_get_current(void)
{
    return current_status;
}

uint32_t led_status_get_flags(void)
{
    return current_flags;
}

esp_err_t led_status_blink(led_status_t status, uint32_t count, uint32_t period_ms)
{
    if (count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    blink_status = status;
    blink_count_remaining = count * 2; // *2 because we toggle on/off
    
    // Update timer period
    xTimerChangePeriod(blink_timer, pdMS_TO_TICKS(period_ms / 2), 0);
    
    // Start blinking
    xTimerStart(blink_timer, 0);
    
    ESP_LOGI(TAG, "Starting blink pattern: %d blinks, %lu ms period", count, period_ms);
    return ESP_OK;
}
