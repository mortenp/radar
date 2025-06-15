#include <stdio.h>
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "ld2410.h"
//#include "rgb.h"
//#include "led_strip.h"
//#include "driver/rmt.h"
#include "esp_mac.h"
#include "esp_err.h"
#include "sdkconfig.h"

////// LED
 #include "led_strip.h"
 #include "led_strip_rmt.h"
 #include "math.h"


 ////// pins
#include "driver/gpio.h"
// polling gpio 
// How often (ms) to poll
static constexpr TickType_t POLL_PERIOD_MS = 500;
// Prototype for your action
void pin_triggered_function(gpio_num_t pin);    
using gpio_cb_t = void(*)(gpio_num_t pin);
// Maximum GPIO number on the S3
static constexpr int MAX_GPIO = 49;


 #define WS2812_GPIO         48
 #define WS2812_LED_COUNT     1
 
static led_strip_handle_t led_strip = NULL;

static const char *TAG = "WALKING_AID";

// Hardware pins
#define LD2410_TX_PIN           36  // ESP32-S3 TX to LD2410 RX
#define LD2410_RX_PIN           37  // ESP32-S3 RX to LD2410 TX  
#define LD2410_PRESENCE_PIN     GPIO_NUM_35  // LD2410 OUT pin
//#define LD2410_UART_BAUD_RATE 115200
// Piezo beeper pin
#define PIEZO_BEEPER_PIN        GPIO_NUM_2   // PWM pin for piezo beeper

// Status indication
//#define STATUS_LED_PIN          GPIO_NUM_38  // RGB LED pin

// Detection parameters
#define MOVING_AVERAGE_WINDOW_SIZE  15

// PWM configuration for piezo beeper
#define BEEPER_LEDC_TIMER       LEDC_TIMER_0
#define BEEPER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BEEPER_LEDC_CHANNEL     LEDC_CHANNEL_0
#define BEEPER_DUTY_RESOLUTION  LEDC_TIMER_10_BIT  // 0-1023 duty range

// Audio alert parameters
#define BEEP_DUTY_CYCLE         512  // 50% duty cycle for clear tone


  int ALERT_IMMEDIATE_LIMIT = 50;
  int ALERT_CLOSE_LIMIT  = 80;
  int ALERT_MEDIUM_LIMIT  = 150;
  int ALERT_FAR_LIMIT  = 200;


#define CONFIDENCE_INCREMENT   3  // Increased from +5
#define CONFIDENCE_DECREMENT   1  // Reduced from -2
#define CONFIDENCE_THRESHOLD  80  // Lowered from 60% (for faster response)
  

#define HYSTERESIS 15  // cm
  /*
  
{"version": 2, "values": {"LOG_DEFAULT_LEVEL_INFO": false, "LOG_DEFAULT_LEVEL_DEBUG": true, "LOG_DEFAULT_LEVEL": 4, "LOG_MAXIMUM_EQUALS_DEFAULT": true}, "ranges": {"LWIP_TCPIP_TASK_STACK_SIZE": [2560, 65536]}, "visible": {"LOG_MAXIMUM_LEVEL_DEBUG": false}}


{"version": 2, "set": { "LOG_MAXIMUM_LEVEL_VERBOSE": true }}

Set LOG_MAXIMUM_LEVEL_VERBOSE
{"version": 2, "values": {"LOG_MAXIMUM_EQUALS_DEFAULT": false, "LOG_MAXIMUM_LEVEL_VERBOSE": true, "LOG_MAXIMUM_LEVEL": 5}, "ranges": {}, "visible": {}}

void esp_log_level_set(const char *tag, esp_log_level_t level)

ESP_LOG_ERROR
ESP_LOG_WARN
ESP_LOG_INFO
ESP_LOG_DEBUG
ESP_LOG_VERBOSE
*/
//esp_log_level_set(TAG, ESP_LOG_DEBUG);

//esp_log_level_set(TAG, ESP_LOG_DEBUG);
//esp_log_level_set("*", ESP_LOG_DEBUG);        // set all components to ERROR level
//esp_log_level_set("wifi", ESP_LOG_WARN);      // enable WARN logs from WiFi stack
//esp_log_level_set("dhcpc", ESP_LOG_INFO);     // enable INFO logs from DHCP client
//esp_log_set_level_master(ESP_LOG_DEBUG);             // enables all INFO logs globally




///// polling gpio version //////
// Task that polls one pin
static void poll_pin_task(void* arg)
{
    gpio_num_t pin = (gpio_num_t)(uintptr_t)arg;
    for (;;) {
        int level = gpio_get_level(pin);
        if (level == 0) {
            ESP_LOGI("POLL", "Pin %d is LOW → triggering", pin);
            pin_triggered_function(pin);
            // wait for it to go back HIGH before retriggering
            while (gpio_get_level(pin) == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}
///// polling gpio version //////
// Call this once to configure the pin and start polling
esp_err_t init_polled_input(gpio_num_t pin, bool pullup = true)
{
    // 1) configure as input (no interrupt)
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask   = 1ULL << pin;
    io_conf.mode           = GPIO_MODE_INPUT;
    io_conf.pull_up_en     = pullup ? GPIO_PULLUP_ENABLE  : GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en   = pullup ? GPIO_PULLDOWN_DISABLE: GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type      = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 2) spawn a task that will poll it
    BaseType_t ok = xTaskCreate(
        poll_pin_task,
        "poll_pin",
        2048,
        (void*)(uintptr_t)pin,
        tskIDLE_PRIORITY + 1,
        nullptr
    );
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

// end polling version

//static led_strip_handle_t led_strip;
///////////////////// LED /////////////////

 void rgb_led_init() {
     esp_err_t err;
 
     led_strip_rmt_config_t rmt_config = {
     .clk_src = RMT_CLK_SRC_DEFAULT,
     .resolution_hz = 10 * 1000 * 1000,  // 10 MHz resolution
     .mem_block_symbols = 64,
     .flags = {
         .with_dma = false,
     },
 };
 
 /*
     led_strip_config_t strip_config = {
         .strip_gpio_num = WS2812_GPIO,
         .max_leds = WS2812_LED_COUNT, // is set to 1
         .led_model = LED_MODEL_WS2812,
 //        .led_pixel_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
         .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB,
         
         .flags = {
             .invert_out = false,
         },
     };
 
     */
     led_strip_config_t strip_config = {
         .strip_gpio_num = WS2812_GPIO,
         .max_leds = WS2812_LED_COUNT, // is set to 1
         .led_model = LED_MODEL_WS2812,
         .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
         .flags = {
             .invert_out = false,
         },
     };
 
   //  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
 
   
    err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
 
     if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "LED strip init failed: %s", esp_err_to_name(err));
     } else {
       //  ESP_LOGI("LED", "LED strip init successful: %s", esp_err_to_name(err));
     ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
 }
 
 enum class LedColor {
     OFF,
     RED,
     GREEN,
     BLUE,
     YELLOW,
     CYAN,
     MAGENTA,
     WHITE,
     BLACK
 };
 
 /*
 void set_led_color(LedColor color) {
     uint8_t r = 0, g = 0, b = 0;
     int strength = 100;
     switch (color) {
         case LedColor::OFF:     r = g = b = 0; break;
         case LedColor::RED:     r = strength; break;
         case LedColor::GREEN:   g = strength; break;
         case LedColor::BLUE:    b = strength; break;
         case LedColor::YELLOW:  r = g = strength; break;
         case LedColor::CYAN:    g = b = strength; break;
         case LedColor::MAGENTA: r = b = strength; break;
         case LedColor::WHITE:   r = g = b = strength; break;
         case LedColor::BLACK:   r = g = b = 0; break;
     }
 
     led_strip_set_pixel(led_strip, 0, r, g, b);
     led_strip_refresh(led_strip);
 }
 */
 
 void rgb_led_off() {
     esp_err_t  err =  led_strip_clear(led_strip);
 
     if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "rgb_led_off failed: %s", esp_err_to_name(err));
     } else {
   //      ESP_LOGI("LED", "rgb_led_off successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
 
 }
 
 
 void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
     esp_err_t  err ;
 
     if (led_strip == nullptr) {
         ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
         return;
     }
 
       err =  led_strip_set_pixel(led_strip, 0, r, g, b);
       if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "led_strip_set_pixel failed: %s", esp_err_to_name(err));
     } else {
        // ESP_LOGI("LED", "led_strip_set_pixel successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
     
     /*
     if (led_strip == nullptr) {
         ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
         return;
     }
 */
       err =  led_strip_refresh(led_strip);
       if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "led_strip_refresh failed: %s", esp_err_to_name(err));
     } else {
       //  ESP_LOGI("LED", "led_strip_refresh successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
 }
 
 
 uint8_t gamma_correct(uint8_t val) {
     return (uint8_t)(std::powf(val / 255.0f, 2.2f) * 255.0f);
 }
 
 void rgb_led_set_color_with_brightness(uint8_t r, uint8_t g, uint8_t b, float brightness) {
     if (brightness < 0.0f) brightness = 0.0f;
     if (brightness > 1.0f) brightness = 1.0f;
 
     uint8_t scaled_r = gamma_correct((uint8_t)(r * brightness));
     uint8_t scaled_g = gamma_correct((uint8_t)(g * brightness));
     uint8_t scaled_b = gamma_correct((uint8_t)(b * brightness));
 
     led_strip_set_pixel(led_strip, 0, scaled_r, scaled_g, scaled_b);
     led_strip_refresh(led_strip);
 }
 
 void rgb_led_blink(LedColor color, int times, int ms_delay, float strength_val) {
     uint8_t r = 0, g = 0, b = 0;
 
     switch (color) {
         case LedColor::RED:      r = 255; g = 0;   b = 0;   break;
         case LedColor::GREEN:    r = 0;   g = 255; b = 0;   break;
         case LedColor::BLUE:     r = 0;   g = 0;   b = 255; break;
         case LedColor::YELLOW:   r = 255; g = 255; b = 0;   break;
         case LedColor::CYAN:     r = 0;   g = 255; b = 255; break;
         case LedColor::MAGENTA:  r = 180; g = 100;   b = 255; break;
         case LedColor::WHITE:    r = 255; g = 255; b = 255; break;
         case LedColor::BLACK:    r = 0;   g = 0;   b = 0;   break;
         default:                 r = 0;   g = 0;   b = 0;   break;
     }
 
     for (int i = 0; i < times; ++i) {
         rgb_led_set_color_with_brightness(r, g, b, strength_val);
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
         rgb_led_off();
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
     }
 }
 



typedef enum {
    ALERT_SILENT = 0,
    ALERT_FAR,          // 300-600cm: Low freq, slow beeps
    ALERT_MEDIUM,       // 150-300cm: Medium freq, medium beeps  
    ALERT_CLOSE,        // 50-150cm: High freq, fast beeps
    ALERT_IMMEDIATE     // 0-50cm: Very high freq, rapid beeps
} alert_level_t;

/*
typedef struct {
    uint32_t frequency_hz;      // Beep frequency
    uint32_t beep_duration_ms;  // How long each beep lasts
    uint32_t beep_interval_ms;  // Time between beeps

    const char* description;
} alert_config_t;

// Alert configurations - frequencies increase with urgency

static const alert_config_t alert_configs[] = {
    {0,    0,   0,    "Silent"},           // ALERT_SILENT
    {800,  30,  2000, "Far"},             // ALERT_FAR: 800Hz, 50ms beep every 2s
    {1200, 50,  1000, "Medium"},          // ALERT_MEDIUM: 1200Hz, 80ms beep every 1s
    {1800, 80, 700,  "Close"},           // ALERT_CLOSE: 1800Hz, 100ms beep every 500ms
    {2500, 100, 300,  "Immediate"}        // ALERT_IMMEDIATE: 2500Hz, 150ms beep every 200ms
};
*/
typedef struct {
    uint32_t frequency_hz;      // Beep frequency in Hz
    uint32_t beep_duration_ms;  // Duration of each beep in ms
    uint32_t beep_interval_ms;  // Time between beeps in ms
    const char* description;     // Description of the alert level
    LedColor led_color;          // LED color for the alert
    int led_blink_times;         // Number of LED blinks
    int led_blink_delay_ms;      // Delay between LED blinks in ms
    float led_strength_val;      // LED brightness level (1.0 = full brightness)
} alert_config_t;

static const alert_config_t alert_configs[] = {
    {0,    0,   0,    "Silent",       LedColor::BLACK,  0, 0, 0.0}, // ALERT_SILENT
    {800,  50,  3000, "Far",         LedColor::GREEN,   1, 500, 0.5}, // ALERT_FAR
    {1200, 50,  2000, "Medium",      LedColor::YELLOW, 1, 300, 0.5}, // ALERT_MEDIUM
    {1800, 50,  1000,  "Close",       LedColor::RED,    1, 200, 0.5}, // ALERT_CLOSE
    {2500, 70, 500,  "Immediate",   LedColor::WHITE,  1, 100, 0.5}  // ALERT_IMMEDIATE
};

typedef struct {
    alert_level_t current_level;
    bool is_beeping;
    uint32_t beep_start_time;
    uint32_t last_beep_time;
    uint32_t beep_count;
} beeper_state_t;

static beeper_state_t beeper = {};

// Initialize piezo beeper PWM

esp_err_t init_piezo_beeper(void) {
    // Configure LEDC timer
    ledc_timer_config_t timer_config = {
        .speed_mode = BEEPER_LEDC_MODE,  
        .duty_resolution = BEEPER_DUTY_RESOLUTION,
        .timer_num = BEEPER_LEDC_TIMER,
         .freq_hz = 1000,  // Default frequency, will be changed dynamically
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));


    // Configure LEDC channel
    ledc_channel_config_t channel_config = {
         .gpio_num = PIEZO_BEEPER_PIN,
             .speed_mode = BEEPER_LEDC_MODE,
        .channel = BEEPER_LEDC_CHANNEL,
   //     .intr_type
        .timer_sel = BEEPER_LEDC_TIMER,
        .duty = 0,  // Start silent
        .hpoint = 0,
        
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    ESP_LOGI(TAG, "Piezo beeper initialized on GPIO%d", PIEZO_BEEPER_PIN);
    return ESP_OK;
}

// Set beeper frequency and enable/disable
void set_beeper_tone(uint32_t frequency_hz, bool enable) {
    if (enable && frequency_hz > 0) {
        // Set frequency
        ledc_set_freq(BEEPER_LEDC_MODE, BEEPER_LEDC_TIMER, frequency_hz);
        // Set duty cycle for audible tone
        ledc_set_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL, BEEP_DUTY_CYCLE);
        ledc_update_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL);
    } else {
        // Silence the beeper
        ledc_set_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL, 0);
        ledc_update_duty(BEEPER_LEDC_MODE, BEEPER_LEDC_CHANNEL);
    }
}


// Determine alert level based on distance
alert_level_t get_alert_level(uint16_t distance_cm) {
    if (distance_cm <= ALERT_IMMEDIATE_LIMIT ) { //+ HYSTERESIS
        return ALERT_IMMEDIATE;
    } else if (distance_cm <= ALERT_CLOSE_LIMIT ) {
        return ALERT_CLOSE;
    } else if (distance_cm <= ALERT_MEDIUM_LIMIT ) {
        return ALERT_MEDIUM;
    } else if (distance_cm <= ALERT_FAR_LIMIT ) {
        return ALERT_FAR;
    } else {
        return ALERT_SILENT;
    }
}

// Update beeper based on current alert level
//void update_beeper_alerts(alert_level_t new_level) {
    void update_beeper_alerts(alert_level_t new_level, ld2410_sensor_t *sensor) {

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if alert level changed
    if (new_level != beeper.current_level) {
        beeper.current_level = new_level;
        beeper.last_beep_time = 0;  // Reset timing to start new pattern immediately
        beeper.is_beeping = false;
        set_beeper_tone(0, false);  // Stop current beep
        
ESP_LOGI(TAG, 
    "🚨 ALERT: %s → %s\n"
    "   Distance: Raw=%dcm | Avg=%dcm\n"
    "   Thresholds: Far=%dcm | Med=%dcm | Close=%dcm\n"
    "   Energy: Move=%d%% | Still=%d%%",
    alert_configs[beeper.current_level].description,
    alert_configs[new_level].description,
    sensor->target_data.raw_detection_distance,
    sensor->stationary_distance_avg.average,
    ALERT_FAR_LIMIT,
    ALERT_MEDIUM_LIMIT,
    ALERT_CLOSE_LIMIT,
    sensor->target_data.moving_target_energy,
    sensor->target_data.stationary_target_energy
);          
 //       ESP_LOGI(TAG, "🔊 Alert level changed to: %s", alert_configs[new_level].description);

  
 

}
    
   const alert_config_t *config = &alert_configs[beeper.current_level];

    // Handle silent mode
    if (beeper.current_level == ALERT_SILENT) {
        if (beeper.is_beeping) {
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
        }
        return;
    }
    
    // Handle beeping pattern
    if (beeper.is_beeping) {
        // Check if current beep should end
        if (current_time - beeper.beep_start_time >= config->beep_duration_ms) {
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
            beeper.last_beep_time = current_time;
            beeper.beep_count++;
        }
    } else {
        // Check if it's time for next beep
        if (current_time - beeper.last_beep_time >= config->beep_interval_ms) {
            set_beeper_tone(config->frequency_hz, true);
            beeper.is_beeping = true;
            beeper.beep_start_time = current_time;
                // Call rgb_led_blink with parameters from config
  //   ESP_LOGI(TAG, "Blinking");
 //  rgb_led_blink(config->led_color, config->led_blink_times, config->led_blink_delay_ms, config->led_strength_val);

        }
    }
}


void load_safe_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
    
    // MINIMAL SAFE CONFIGURATION
    config->engineering_mode = false;        // Stay in normal mode
    config->bluetooth_enabled = false;       // Keep bluetooth off
    config->light_function_enabled = false;  // Keep light function off
    
    // Set high precision distance resolution for walking aid
    config->distance_resolution = DISTANCE_RESOLUTION_0_2;  // 0.2m precision
    
    // Only adjust a few gate sensitivities - be conservative
    config->max_move_distance_gate = 8;      // Use all gates
    config->max_still_distance_gate = 8;     // Slightly shorter for stationary
    
    // Conservative sensitivity adjustments (small changes from defaults)
    for (int gate = 0; gate < 3; gate++) {
        config->move_thresholds[gate] = 50;   // Slightly higher for close range
        config->still_thresholds[gate] = 0;
    }
    for (int gate = 3; gate < LD2410_MAX_GATES; gate++) {
        config->move_thresholds[gate] = 40;   // Default values for far range
        config->still_thresholds[gate] = 30;
    }
    
    ESP_LOGI(TAG, "Loaded SAFE configuration (0.2m resolution, minimal changes)");
}

void load_minimal_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
    
    // Minimal changes - just disable engineering mode
    config->engineering_mode = false;
    config->bluetooth_enabled = false;
    config->light_function_enabled = false;
    
    // Don't change sensitivities - use sensor defaults
    ESP_LOGI(TAG, "Loaded MINIMAL profile (sensor defaults)");
}

// Configuration profiles for different scenarios
void load_high_sensitivity_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
 
ALERT_IMMEDIATE_LIMIT = 50;
ALERT_CLOSE_LIMIT  = 80;
ALERT_MEDIUM_LIMIT  = 150;
ALERT_FAR_LIMIT  = 200;



    // Override for maximum sensitivity (good for testing/quiet environments)
    for (int gate = 0; gate < LD2410_MAX_GATES; gate++) {
        config->move_thresholds[gate] = 0;
        config->still_thresholds[gate] = 0;
    }
    config->engineering_mode = true;  // For debugging
    ESP_LOGI(TAG, "Loaded HIGH SENSITIVITY profile");
}

void load_balanced_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
    
config->distance_resolution =  DISTANCE_RESOLUTION_0_2; //DISTANCE_RESOLUTION_0_75 or DISTANCE_RESOLUTION_0_2

ld2410_set_max_distances (8, 8);
//max_moving_gate, max_stationary_gate
ALERT_IMMEDIATE_LIMIT = 40;
ALERT_CLOSE_LIMIT  = 60;
ALERT_MEDIUM_LIMIT  = 150;
ALERT_FAR_LIMIT  = 300; 

    // Balanced sensitivity for normal use
    for (int gate = 0; gate < 3; gate++) {
        config->move_thresholds[gate] = 50;
        config->still_thresholds[gate] = 0;
    }


    for (int gate = 3; gate < 6; gate++) {
        config->move_thresholds[gate] = 40;
        config->still_thresholds[gate] = 40;
    }
    for (int gate = 6; gate < LD2410_MAX_GATES; gate++) {
        config->move_thresholds[gate] = 30;
        config->still_thresholds[gate] = 15;
    }
    config->engineering_mode = false;  // Normal operation
    ESP_LOGI(TAG, "Loaded BALANCED profile");
}

void load_longrange_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
    
config->distance_resolution =  DISTANCE_RESOLUTION_0_75; //DISTANCE_RESOLUTION_0_75 or DISTANCE_RESOLUTION_0_2

ALERT_IMMEDIATE_LIMIT = 60;
ALERT_CLOSE_LIMIT  = 100;
ALERT_MEDIUM_LIMIT  = 200;
ALERT_FAR_LIMIT  = 300;


ld2410_set_max_distances (8, 8);
//max_moving_gate, max_stationary_gate

    // Balanced sensitivity for normal use
    for (int gate = 0; gate < 3; gate++) {
        config->move_thresholds[gate] = 50;
        config->still_thresholds[gate] = 0;
    }


    for (int gate = 3; gate < 6; gate++) {
        config->move_thresholds[gate] = 40;
        config->still_thresholds[gate] = 40;
    }
    for (int gate = 6; gate < LD2410_MAX_GATES; gate++) {
        config->move_thresholds[gate] = 30;
        config->still_thresholds[gate] = 15;
    }
    config->engineering_mode = false;  // Normal operation
    ESP_LOGI(TAG, "Loaded BALANCED profile");
}


void load_low_sensitivity_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
    
    // Lower sensitivity for noisy environments
    for (int gate = 0; gate < 3; gate++) {
        config->move_thresholds[gate] = 60;
        config->still_thresholds[gate] = 60;
    }
    for (int gate = 3; gate < 6; gate++) {
        config->move_thresholds[gate] = 45;
        config->still_thresholds[gate] = 45;
    }
    for (int gate = 6; gate < LD2410_MAX_GATES; gate++) {
        config->move_thresholds[gate] = 30;
        config->still_thresholds[gate] = 30;
    }
    ESP_LOGI(TAG, "Loaded LOW SENSITIVITY profile");
}

void load_calibration_config(ld2410_config_t *config) {
    ld2410_load_walking_aid_config(config);  // Start with defaults
    
    config->engineering_mode = true;         // Enable engineering mode
    config->light_function_enabled = true;   // Enable light sensing for testing
    config->light_threshold = 128;           // Default light threshold
    config->bluetooth_enabled = true;        // Enable bluetooth for external config
    
    ESP_LOGI(TAG, "Loaded CALIBRATION profile (engineering mode enabled)");
}

/*
void configure_status_led(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << STATUS_LED_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(STATUS_LED_PIN, 0);
}
*/

void test_multiple_baud_rates(void) {
    uint32_t baud_rates[] = {115200};
    int num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
    
    ESP_LOGI(TAG, "🔍 Testing multiple baud rates to find working one...");
    
    for (int i = 0; i < num_rates; i++) {
        ESP_LOGI(TAG, "Testing baud rate: %lu", baud_rates[i]);
        
        // Reconfigure UART with new baud rate
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        
        esp_err_t ret = uart_param_config(LD2410_UART_NUM, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set baud rate %lu", baud_rates[i]);
            continue;
        }
        
        // Clear buffers and wait
        uart_flush(LD2410_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        // Read data and look for LD2410 frame headers
        uint8_t buffer[512];
        int length = uart_read_bytes(LD2410_UART_NUM, buffer, sizeof(buffer), 1000 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            ESP_LOGI(TAG, "Received %d bytes at %lu baud:", length, baud_rates[i]);
            ESP_LOG_BUFFER_HEX(TAG, buffer, length > 32 ? 32 : length);
            
            // Look for LD2410 frame headers
            bool found_header = false;
            for (int j = 0; j <= length - 4; j++) {
                if (buffer[j] == 0xF4 && buffer[j+1] == 0xF3 && 
                    buffer[j+2] == 0xF2 && buffer[j+3] == 0xF1) {
                    ESP_LOGI(TAG, "✅ Found LD2410 frame header at %lu baud!", baud_rates[i]);
                    found_header = true;
                    break;
                }
            }
            
            if (found_header) {
                ESP_LOGI(TAG, "🎯 Using baud rate %lu", baud_rates[i]);
                return;  // Keep this baud rate
            } else {
                ESP_LOGW(TAG, "❌ No valid LD2410 headers found at %lu baud", baud_rates[i]);
            }
        } else {
            ESP_LOGI(TAG, "No data at %lu baud", baud_rates[i]);
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Restore original baud rate if nothing worked
    ESP_LOGW(TAG, "⚠️ No valid baud rate found, restoring 256000");
    uart_config_t uart_config = {
        .baud_rate = LD2410_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(LD2410_UART_NUM, &uart_config);
    uart_flush(LD2410_UART_NUM);
}

// Static variable maintains state between calls
static uint8_t target_confidence = 0;

void process_sample(ld2410_sensor_t *sensor) {
    // 1. More tolerant validation
    bool is_valid = (sensor->target_data.target_state != LD2410_TARGET_NONE) && 
                   (MAX(sensor->target_data.moving_target_energy,
                       sensor->target_data.stationary_target_energy) > 10); // Lowered from 15%

    // 2. Smoother confidence update
    sensor->target_confidence = CLAMP(
        sensor->target_confidence + (is_valid ? CONFIDENCE_INCREMENT : -CONFIDENCE_DECREMENT),
        0,
        100
    );

    // 3. New: Minimum confidence for recent valid readings
    static uint8_t valid_streak = 0;
    if (is_valid) {
        valid_streak = MIN(valid_streak + 1, 5);
        sensor->target_confidence = MAX(sensor->target_confidence, valid_streak * 10);
    } else {
        valid_streak = 0;
    }

    // 4. Confirm target with lower threshold
    sensor->target_confirmed = (sensor->target_confidence >= CONFIDENCE_THRESHOLD);
}

void sensor_task(void *pvParameters) {
    ld2410_sensor_t sensor = {0};
    ld2410_config_t config = {0};
    uint32_t uart_debug_counter = 0;
    uint32_t last_sensor_read = 0;
    const uint32_t SENSOR_READ_INTERVAL = 100; // 100ms = 10Hz
    
    ESP_LOGI(TAG, "Starting sensor task");
    
    // Initialize moving averages
    esp_err_t ret = ld2410_init_moving_averages(&sensor, MOVING_AVERAGE_WINDOW_SIZE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize moving averages: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize presence detection pin
    ret = ld2410_init_presence_pin(&sensor, LD2410_PRESENCE_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize presence detection pin: %s", esp_err_to_name(ret));
        ld2410_cleanup_moving_averages(&sensor);
        vTaskDelete(NULL);
        return;
    }
    
    // Wait for sensor to stabilize
    ESP_LOGI(TAG, "Waiting for sensor to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(3000));  // Longer wait
    
    // Test current baud rate first
        ESP_LOGI(TAG, "Testing initial sensor communication...");
        uint8_t initial_buffer[256];
        int initial_length = uart_read_bytes(LD2410_UART_NUM, initial_buffer, sizeof(initial_buffer), 1000 / portTICK_PERIOD_MS);
    
    if (initial_length > 0) {
        ESP_LOGI(TAG, "Initial data (%d bytes):", initial_length);
        ESP_LOG_BUFFER_HEX(TAG, initial_buffer, initial_length > 32 ? 32 : initial_length);
        
        // Check if we have valid LD2410 frames
        bool valid_frames = false;
        for (int i = 0; i <= initial_length - 4; i++) {
            if (initial_buffer[i] == 0xF4 && initial_buffer[i+1] == 0xF3 && 
                initial_buffer[i+2] == 0xF2 && initial_buffer[i+3] == 0xF1) {
                ESP_LOGI(TAG, "✅ Valid LD2410 frames detected!");
                valid_frames = true;
                break;
            }
        }
        
        if (!valid_frames) {
            ESP_LOGW(TAG, "❌ Invalid frame format detected, testing baud rates...");
            rgb_led_blink(LedColor::BLUE, 1, 100, 255);
            vTaskDelay(pdMS_TO_TICKS(1000));
            test_multiple_baud_rates();
        }
    } else {
        ESP_LOGW(TAG, "❌ No initial data, testing baud rates...");
            rgb_led_blink(LedColor::BLUE, 2, 100, 255);
            vTaskDelay(pdMS_TO_TICKS(1000));
            test_multiple_baud_rates();
    }
    
    // Try basic sensor commands to get it into proper mode
    ESP_LOGI(TAG, "🔄 Attempting to reset sensor to known state...");
    
    // Try enable/disable config mode to reset sensor
    ld2410_enable_config_mode();
    vTaskDelay(pdMS_TO_TICKS(500));
    ld2410_disable_config_mode();
    vTaskDelay(pdMS_TO_TICKS(1000));
    

    // Apply SAFE configuration
    ESP_LOGI(TAG, "🔧 Applying safe sensor configuration...");
 //   load_safe_config(&config);

   load_balanced_config(&config);
 //   load_longrange_config(&config);
 
    config.max_move_distance_gate = 6;  // Reduce range for better focus
    config.move_thresholds[0] = 60;     // Higher sensitivity near sensor


 

    ret = ld2410_apply_config_safely(&config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Configuration may have failed, continuing with defaults");
        rgb_led_blink(LedColor::BLUE, 3, 100, 255);
    }
    

    // Clear buffers
    uart_flush(LD2410_UART_NUM);
    
    // Final test
    ESP_LOGI(TAG, "Final communication test...");
    uint8_t final_buffer[256];
    int final_length = uart_read_bytes(LD2410_UART_NUM, final_buffer, sizeof(final_buffer), 2000 / portTICK_PERIOD_MS);
    
    if (final_length > 0) {
        ESP_LOGI(TAG, "Final test data (%d bytes):", final_length);
        ESP_LOG_BUFFER_HEX(TAG, final_buffer, final_length > 32 ? 32 : final_length);
        
        // Analyze the data
        bool found_valid = false;
        for (int i = 0; i <= final_length - 4; i++) {
            if (final_buffer[i] == 0xF4 && final_buffer[i+1] == 0xF3 && 
                final_buffer[i+2] == 0xF2 && final_buffer[i+3] == 0xF1) {
                ESP_LOGI(TAG, "✅ Sensor communication restored!");
                rgb_led_blink(LedColor::GREEN, 3, 100, 255);
                   // Startup beep sequence
    for (int i = 0; i < 3; i++) {
        set_beeper_tone(1000, true);
        vTaskDelay(pdMS_TO_TICKS(100));
        set_beeper_tone(0, false);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

                found_valid = true;
                break;
            }
        }
        
        if (!found_valid) {

        set_beeper_tone(440, true);
        vTaskDelay(pdMS_TO_TICKS(1000));
        set_beeper_tone(0, false);

            rgb_led_blink(LedColor::RED, 3, 100, 255);
            ESP_LOGE(TAG, "❌ Sensor still not communicating properly");
            ESP_LOGE(TAG, "Try:");
            ESP_LOGE(TAG, "1. Power cycle the sensor");
            ESP_LOGE(TAG, "2. Check wiring connections");
            ESP_LOGE(TAG, "3. Verify sensor power supply");
        }
    }
    
    ESP_LOGI(TAG, "🚀 Starting walking aid main loop...");
    
     
    while (1) {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Read sensor data at fixed 100ms intervals
        if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL) {
            last_sensor_read = current_time;
            

            ret = ld2410_read_data(&sensor);
            
            if (ret == ESP_OK) {

                process_sample(&sensor); // new
 //               ESP_LOGI(TAG,"ld2410_read_data OK");

//ESP_LOGI(TAG, "sensor.target_data.detection_distance: %d" , sensor.target_data.detection_distance);

                // Sensor data processing (same as before)
                uint16_t primary_distance = sensor.target_data.raw_detection_distance;
                const char* obstacle_type = "None";
                
                if (sensor.target_data.target_state == 1) {
                    primary_distance = sensor.target_data.moving_target_distance;
                    obstacle_type = "Moving";
                } else if (sensor.target_data.target_state == 2) {
                    primary_distance = sensor.target_data.stationary_target_distance;
                    obstacle_type = "Stationary";
                } else if (sensor.target_data.target_state == 3) {
                    primary_distance = (sensor.target_data.moving_target_distance < sensor.target_data.stationary_target_distance) ?
                                     sensor.target_data.moving_target_distance : sensor.target_data.stationary_target_distance;
                    obstacle_type = "Both";
                }
          
                alert_level_t alert_level = ALERT_FAR;

         //       alert_level_t alert_level = get_alert_level(primary_distance);
         if (sensor.target_confirmed) {

            alert_level_t alert_level = get_alert_level(sensor.detection_distance_avg.average);

            if ( sensor.target_data.stationary_target_energy > CONFIDENCE_THRESHOLD && sensor.target_data.moving_target_energy < CONFIDENCE_THRESHOLD ){
 //               ESP_LOGD(TAG, "get_alert_level using stationary_distance_avg %d", sensor.stationary_distance_avg.average);
                 alert_level_t alert_level = get_alert_level(sensor.stationary_distance_avg.average);
            }
                        if ( sensor.target_data.moving_target_energy > CONFIDENCE_THRESHOLD){
//                           ESP_LOGD(TAG, "get_alert_level using moving_distance_avg %d", sensor.moving_distance_avg.average);
                 alert_level_t alert_level = get_alert_level(sensor.moving_distance_avg.average);
            }
            

         //       alert_level_t alert_level = get_alert_level(sensor.stationary_distance_avg.average);
                update_beeper_alerts(alert_level, &sensor);
         }


                const char* alert_emoji = "🟢";
                const char* alert_name = "CLEAR";
                switch (alert_level) {
                    case ALERT_IMMEDIATE: alert_emoji = "🔴"; alert_name = "IMMEDIATE"; break;
                    case ALERT_CLOSE:     alert_emoji = "🟠"; alert_name = "CLOSE"; break;
                    case ALERT_MEDIUM:    alert_emoji = "🟡"; alert_name = "MEDIUM"; break;
                    case ALERT_FAR:       alert_emoji = "🔵"; alert_name = "FAR"; break;
                    default:              alert_emoji = "🟢"; alert_name = "CLEAR"; break;
                }
 /*              
                // When alert level changes:
ESP_LOGI(TAG, "🚨 ALERT: %s (Distance: %3dcm → Avg: %3dcm ±%dcm)", 
         alert_configs[alert_level].description, 
         sensor.target_data.stationary_target_distance,
         sensor.stationary_distance_avg.average,
         abs(sensor.target_data.stationary_target_distance - sensor.stationary_distance_avg.average));
*/ 
           //     ESP_LOGI(TAG, "📍 %s %s - %s at %dcm | Beeps: %lu", 
           //              alert_emoji, alert_name, obstacle_type, primary_distance, beeper.beep_count);

           uart_debug_counter++;

                if (uart_debug_counter >= 20) {
                ESP_LOGI(TAG, "Mov: %d cm %d%% | Still: %d cm %d%% | det: %d cm, Pres: %s | st avg: %d | mov avg: %d | det avg: %d | avg count %d", 
                         sensor.target_data.moving_target_distance, 
                         sensor.target_data.moving_target_energy,
                         sensor.target_data.stationary_target_distance, 
                         sensor.target_data.stationary_target_energy,
                         sensor.target_data.raw_detection_distance,
                         sensor.presence_detected ? "HIGH" : "LOW",
                         sensor.stationary_distance_avg.average,
                         
                         sensor.moving_distance_avg.average,
                         sensor.detection_distance_avg.average,
                      //   sensor.stationary_distance_avg.is_full ? "FULL" : "No",
                         sensor.stationary_distance_avg.count
                );
                 uart_debug_counter = 0;
            }

            } else if (ret == ESP_ERR_NOT_FOUND) {
                
                ESP_LOGD(TAG, "No sensor data parsed");
                rgb_led_blink(LedColor::RED, 1, 50, 255);
                vTaskDelay(pdMS_TO_TICKS(1000));
            } else if (ret != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Sensor read error: %s", esp_err_to_name(ret));
                rgb_led_blink(LedColor::RED, 2, 50, 255);
            }
            
            if (sensor.presence_pin_enabled) {
            //     ESP_LOGI(TAG,"ld2410_read_presence_pin now");
                ld2410_read_presence_pin(&sensor);
            }
        }
        
        update_beeper_alerts(beeper.current_level, &sensor);
        

        // Statistics
           /*
        
     
        if (uart_debug_counter >= 2000) {
            ESP_LOGI(TAG, "📊 5s Stats: Frames %lu | Success %.1f%%", 
                     sensor.total_frames_received,
                     sensor.total_frames_received > 0 ? 
                     (float)sensor.valid_frames_parsed / sensor.total_frames_received * 100.0f : 0.0f);
            uart_debug_counter = 0;
        }
      */  
      //  vTaskDelay(pdMS_TO_TICKS(10));
    }
    
   ld2410_cleanup_moving_averages(&sensor);
}


///// polling gpio version //////
// Example “action” when pin goes low
void pin_triggered_function(gpio_num_t pin)
{

    
    if ( (uint8_t) pin == 4){
      //  save_nvs(nvs_var_serial, 0);
        // engineering mode
    
  ESP_LOGI(TAG, "🔧 Applying high_sensitivity_config");

  ld2410_config_t config = {0};

   // Try basic sensor commands to get it into proper mode
 //   ESP_LOGI(TAG, "🔄 Attempting to reset sensor to known state...");
    
    // Try enable/disable config mode to reset sensor
 //   ld2410_enable_config_mode();
  //  vTaskDelay(pdMS_TO_TICKS(500));
  
    // Apply SAFE configuration
  
 //   load_safe_config(&config);

 //   load_balanced_config(&config);
    load_high_sensitivity_config(&config);


 esp_err_t ret = ld2410_apply_config_safely(&config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Configuration may have failed, continuing with defaults");
    }
    
 // ld2410_disable_config_mode();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    }
    
    ESP_LOGI("ACTION", "Handling event from pin %d", pin);
}



extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting LD2410 Walking Aid with Audio Alerts");
    ESP_LOGI(TAG, "ESP32-S3 with Piezo Beeper");
    
    // Set log level
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("LD2410", ESP_LOG_INFO);
    
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    

    //rgb_led_init();
 rgb_led_init();
 
 ESP_ERROR_CHECK(init_polled_input(GPIO_NUM_4, true));


    // Initialize piezo beeper
    esp_err_t ret = init_piezo_beeper();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize piezo beeper");
       //   rgb_led_blink(LedColor::RED, 1, 50, 255);
        return;
    }
  

    // Initialize LD2410 sensor
   // ESP_LOGI(TAG, "Initializing LD2410 sensor on TX:%d, RX:%d", LD2410_TX_PIN, LD2410_RX_PIN);
    ret = ld2410_init(LD2410_TX_PIN, LD2410_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2410 sensor: %s", esp_err_to_name(ret));
          rgb_led_blink(LedColor::RED, 1, 50, 255);
        return;
    }
    
            
   //      led_strip_del(led_strip);
 
    ESP_LOGI(TAG, "Creating sensor task...");
    BaseType_t task_created = xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
          rgb_led_blink(LedColor::RED, 2, 50, 255);
        return;
    }
    
    ESP_LOGI(TAG, "🔊 Walking Aid System Started Successfully");
    ESP_LOGI(TAG, "Audio Alert Levels:");
    ESP_LOGI(TAG, "  🔴 IMMEDIATE (0-50cm): %dHz, %dms beeps every %dms", 
             alert_configs[ALERT_IMMEDIATE].frequency_hz,
             alert_configs[ALERT_IMMEDIATE].beep_duration_ms,
             alert_configs[ALERT_IMMEDIATE].beep_interval_ms);
    ESP_LOGI(TAG, "  🟠 CLOSE (50-150cm): %dHz, %dms beeps every %dms", 
             alert_configs[ALERT_CLOSE].frequency_hz,
             alert_configs[ALERT_CLOSE].beep_duration_ms,
             alert_configs[ALERT_CLOSE].beep_interval_ms);
    ESP_LOGI(TAG, "  🟡 MEDIUM (150-300cm): %dHz, %dms beeps every %dms", 
             alert_configs[ALERT_MEDIUM].frequency_hz,
             alert_configs[ALERT_MEDIUM].beep_duration_ms,
             alert_configs[ALERT_MEDIUM].beep_interval_ms);
    ESP_LOGI(TAG, "  🔵 FAR (300-600cm): %dHz, %dms beeps every %dms", 
             alert_configs[ALERT_FAR].frequency_hz,
             alert_configs[ALERT_FAR].beep_duration_ms,
             alert_configs[ALERT_FAR].beep_interval_ms);
    ESP_LOGI(TAG, "Expected wiring:");
    ESP_LOGI(TAG, "  Piezo Beeper -> GPIO%d", PIEZO_BEEPER_PIN);
    ESP_LOGI(TAG, "  LD2410 TX    -> GPIO%d", LD2410_RX_PIN);
    ESP_LOGI(TAG, "  LD2410 RX    -> GPIO%d", LD2410_TX_PIN);
    ESP_LOGI(TAG, "  LD2410 OUT   -> GPIO%d", LD2410_PRESENCE_PIN);
}
