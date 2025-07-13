#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//#include "vl53l5cx_api.h"
#include "vl53l5cx_api.h"
//#include "vl53l0x.h"


#include "esp_log.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
//#include "driver/ledc.h"

#include "driver/i2c.h"

//#include "driver/i2c_master.h"
//#include "driver/i2c_slave.h"

 #include "esp_task_wdt.h"
 #define WATCHDOG_TIMEOUT_MSEC 20  // Reset if no feed within 10 milliseconds

 #include "nvs_flash.h"

/// #include "blink_task.h"
 
////// LED
 #include "led_strip.h"
 #include "led_strip_rmt.h"
 #include "math.h"
 #include <algorithm>

 #include "driver/gpio.h"

 //#include "vl53l1x.h"
//#include "hcsr04_driver.h"
#include <ultrasonic.h>

#define TRIGGER_GPIO GPIO_NUM_4
#define ECHO_GPIO GPIO_NUM_16
#define MAX_DISTANCE_CM 500 // 5m max

#define TRIGGER_GPIO_2 GPIO_NUM_3
#define ECHO_GPIO_2 GPIO_NUM_8

#include <gps.h>

/*
#include "nmea_example.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"
#include "gprmc.h"
#include "gpgsa.h"
#include "gpvtg.h"
#include "gptxt.h"
#include "gpgsv.h"
*/
#include <vector>
#include <memory>
#include <cstdint>

#include "driver/uart.h"

#include "pwm_audio.h"
//#include "driver/i2s.h"
//#include "soc/soc.h" // For ets_delay_us
//#include "driver/sdm.h"

//#include "ets_sys.h"

#include <map>

#include "esp_timer.h"


//import std;

//static bool useAcc = 0;
//#define  useAccelerometer 
//#define useRGBLed

//#define useUltrasound 
//#define useVl53l5cx
//#define useUartA02YYUW
//#define useGPS00 
//#define useGPS_2
//#define useXiaoCam
//#define  useUart 
#define useVibrator2
//#define useAudio
//#define usePinPolling
#define useLuna


 // gyro
#ifdef useAccelerometer
 #include <mpu9250.h>
 #include <math.h> // For atan2f and sqrtf
#endif


// --- Alert Level Types ---
typedef enum {
    ALERT_SILENT = 0,
    ALERT_VERYFAR,
    ALERT_FAR,
    ALERT_MEDIUMFAR,
    ALERT_MEDIUM,
    ALERT_CLOSE,
    ALERT_VERYCLOSE,
    ALERT_IMMEDIATE
} alert_level_t; // Now this is defined

enum class LedColor {
    OFF, RED, ORANGE, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK
};

#ifdef useAccelerometer
// --- Data Structures ---
typedef struct {
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float pitch, roll, temp_c;
} mpu_data_t;
#endif

typedef struct {
    uint32_t frequency_hz, 
    beep_duration_ms, 
    beep_interval_ms;
    const char* description;
    LedColor led_color;
    int led_blink_times, 
    led_blink_delay_ms;
    float led_strength_val;
} alert_config_t;

typedef struct {
    alert_level_t current_level;
    bool is_beeping;
    uint32_t beep_start_time, last_beep_time, beep_count;
} beeper_state_t;

typedef struct {
    alert_level_t level;
    uint32_t duration_ms;
} audio_command_t;

/*
// Audio Control Structure
typedef struct {
    QueueHandle_t queue;
    SemaphoreHandle_t mutex;
    volatile bool active;
    volatile bool initialized;
} audio_state_t;


static audio_state_t audio_state = {
    .queue = NULL,
    .mutex = NULL,
    .active = false,
    .initialized = false
};
*/
// Static allocation for mutex
//static StaticSemaphore_t xMutexBuffer;

#ifdef useAudio
static constexpr uint32_t SAMPLE_RATE = 16000;  // Optimal for ESP32 audio
static constexpr size_t RINGBUF_SIZE = 1024 * 4; // 4KB ring buffer
static constexpr uint32_t DEFAULT_DURATION_MS = 500;
const int BIT_DEPTH = 8;       // Bit depth. 8-bit is standard for PWM audio on ESP32.
static volatile bool audio_active = false;
static alert_level_t current_level = ALERT_SILENT;

    static uint32_t   ultraSensorDistance1;
    static uint32_t  ultraSensorDistance2;


//static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000; // 16kHz
//static constexpr uint32_t DURATION_MS = 500; // 500ms per tone
//static constexpr uint32_t SAMPLES_PER_BUFFER = 128; // Optimal chunk size

// Audio System State
static QueueHandle_t audioQueue = NULL;
static SemaphoreHandle_t audio_mutex = NULL;
//static volatile bool audio_active = false;
#endif

// ----------------------------------------------------------------
// --- NEW: Single LED State Struct and Mode Enum ---
// ----------------------------------------------------------------

#ifdef useRGBLed
// Defines what the WS2812 LED part of the state should be doing
typedef enum {
    LED_MODE_OFF,
    LED_MODE_SOLID,
    LED_MODE_BLINK,
} led_mode_t;
#endif
/*
// This single struct describes the complete desired state for ALL LEDs
typedef struct {
    led_mode_t ws2812_mode; // The mode for the main RGB LED
    uint8_t r, g, b;
    float brightness;
    uint32_t blink_period_ms;

    bool top_led_on;      // The state for the Red GPIO LED
    bool bottom_led_on;   // The state for the Blue GPIO LED

} led_state_t;
*/


// This single struct describes the complete desired state for ALL LEDs.
// We have removed the complex "mode" enum.
// ----------------------------------------------------------------
// --- ACTION: Replace your LED State Struct with this version ---
// ----------------------------------------------------------------
#ifdef useRGBLed
struct led_state_t { // Give the struct its name at the beginning
    uint8_t r = 0, g = 0, b = 0;
    float brightness = 0.0f;
    bool top_led_on = false;
    bool bottom_led_on = false;

    // Now, inside the struct, the name 'led_state_t' is already known.
    // This will compile correctly.
    bool operator!=(const led_state_t& other) const {
        return r != other.r || 
               g != other.g || 
               b != other.b || 
               brightness != other.brightness ||
               top_led_on != other.top_led_on ||
               bottom_led_on != other.bottom_led_on;
    }
};


// The handle for our queue, which will now carry this new struct
QueueHandle_t led_state_queue;
#endif

typedef struct {
    VL53L5CX_Configuration *dev;
    const uint8_t (*gridLayout)[4];
    alert_level_t *gridAlerts;
} sensor_task_params_t;

static const alert_config_t alert_configs[] = {
    {0,    0,   0,    "Silent",    LedColor::BLACK,  1, 100, 0}, // ALERT_SILENT
    {500,  20,  100, "VeryFar",         LedColor::BLUE,   1, 200, 0}, // ALERT_VERYFAR
    {800,  50,  50, "Far",         LedColor::GREEN,   1, 200, 0.6}, // ALERT_FAR
    {1000,  50,  50, "MediumFar",  LedColor::YELLOW,   3, 200, 0.6}, // ALERT_MEDIUMFAR
    {1200, 50,  80, "Medium",      LedColor::ORANGE, 3, 300, 0.7}, // ALERT_MEDIUM
    {1800, 50,  120,  "Close",     LedColor::RED,    5, 200, 0.8}, // ALERT_CLOSE
    {1800, 50,  120,  "`Very Close",     LedColor::RED,    5, 200, 0.8},  //ALERT_VERYCLOSE
    {2500, 70, 200,  "Immediate",  LedColor::WHITE,  5, 100, 0.8}  // ALERT_IMMEDIATE
};

#define RED_PIN     GPIO_NUM_21
#define BLUE_PIN  GPIO_NUM_47
#define GREEN_PIN  GPIO_NUM_45

#define RED_PIN_1     GPIO_NUM_35
#define BLUE_PIN_1  GPIO_NUM_14 // yellow traffic light
#define GREEN_PIN_1  GPIO_NUM_37

// Define GPIO pin number
//#define BLINK_PIN GPIO_NUM_35

#define centreAlert_WarnLevel 3

#define acc_Move_limit 10
static float last_pitch;
static float last_roll;
static bool vl53l5cx_running = false;
static TaskHandle_t xHandle_mpu;
static TaskHandle_t xHandle_mpu_processor;
static TaskHandle_t xHandle_ultrasonic;
static TaskHandle_t xHandle_ultrasonic_2;
static TaskHandle_t xHandle_vl53l5cx;
static TaskHandle_t xHandle_vibrator;
static TaskHandle_t xHandle_blink;
static TaskHandle_t xHandle_GPS;
static TaskHandle_t xHandle_luna;

#ifdef useAudio
static TaskHandle_t audioTaskHandle;
#endif

//TaskHandle_t sensorTaskHandle = NULL;
//TaskHandle_t audioTaskHandle = NULL;
QueueHandle_t audioAlertQueue = NULL;

static SemaphoreHandle_t i2c_mutex = NULL;
static i2c_master_bus_handle_t bus_handle = NULL;
//vl53l0x_dev_t dev_vl53l0x; // Device descriptor for the sensor


// --- Sigma-Delta Audio Configuration for ESP-IDF v5.5 ---
//#define AUDIO_SAMPLE_RATE   (22050)      // Sample rate for audio waveform generation
//#define SDM_CARRIER_FREQ_HZ (1 * 1000 * 1000) // 1 MHz carrier frequency for the modulator
//#define SDM_GPIO_NUM        (GPIO_NUM_5) // The GPIO pin for SDM output
//sdm_channel_handle_t sdm_chan = NULL;


// --- I2S Audio Configuration ---
//#define I2S_SAMPLE_RATE     (16000)
//#define I2S_BUFFER_SAMPLES  (256)
//#define I2S_PORT            (I2S_NUM_0)
//#define DAC_PIN             (GPIO_NUM_25)
// --- Forward Declarations ---
//void audioTask(void *parameter);
void play_tone(float frequency, int duration_ms);
void stop_tone();
void configure_i2s();
//void configure_sdm_audio();


#define movement_timeout_msec 50
static int last_movement = 0;
// Define Notification Bits to act as commands
#define START_BIT   (1 << 0) // Bit 0 for START command
#define STOP_BIT    (1 << 1) // Bit 1 for STOP command

bool vl53l5cx_hasMutex = false;
bool mpu_hasMutex = false;

// #define WS2812_GPIO         48
// #define WS2812_LED_COUNT     1
 
static const char *TAG = "WALKING_AID";

//static led_strip_handle_t led_strip = NULL;
// Piezo beeper pin
#define PIEZO_BEEPER_PIN        GPIO_NUM_38   // PWM pin for piezo beeper
#define VIBRATORPIN GPIO_NUM_10
#define VIBRATORPIN2 GPIO_NUM_11
static bool alwaysBeep = true;

// Detection parameters
#define MOVING_AVERAGE_WINDOW_SIZE  15

// PWM configuration for piezo beeper
#define BEEPER_LEDC_TIMER       LEDC_TIMER_0
#define BEEPER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BEEPER_LEDC_CHANNEL     LEDC_CHANNEL_0
#define BEEPER_DUTY_RESOLUTION  LEDC_TIMER_10_BIT  // 0-1023 duty range

// Audio alert parameters
#define BEEP_DUTY_CYCLE         512  // 50% duty cycle for clear tone
/*
// vl53l1x ///
static VL53L1_Dev_t dev;
#define VLTAG "VL53L1X"
VL53L1_Error status = VL53L1_ERROR_NONE;
VL53L1_RangingMeasurementData_t rangingData;
uint8_t dataReady = 0;
uint16_t range;
*/

/// UART
#define RXD_PIN 7
#define TXD_PIN 6
#define UART_NUM UART_NUM_1
#define UART_BUFFER_SIZE (1024)
#define UART_TIMEOUT_MS 30

#ifdef useGPS
#define UART_GPS UART_NUM_2 
#define UART_GPS_TXD GPIO_NUM_6 
#define UART_GPS_RXD GPIO_NUM_7 
#endif

#ifdef useXiaoCam
#define CAM_PIN        GPIO_NUM_40
#endif

static bool useAudioAlerts = true;
static bool useVibratorAlerts = false;

#define ALERT_TYPE_PIN GPIO_NUM_39


// configure alert distances
int ALERT_IMMEDIATE_LIMIT = 40;
int ALERT_VERYCLOSE_LIMIT  = 60;
int ALERT_CLOSE_LIMIT  = 90;
int ALERT_MEDIUM_LIMIT  = 110;
int ALERT_MEDIUMFAR_LIMIT  = 140;
int ALERT_FAR_LIMIT  = 160;
int ALERT_VERYFAR_LIMIT  = 200;

static bool indoor = false;
static float sensor2_diff = -20.0;

static float fall_magnitude = 1.7;

static uint16_t average_cm = 150;
static uint16_t  total_average_cm = 150;

#define HYSTERESIS 15  // cm

   // Static arrays for grid layout and alerts
    static const uint8_t gridLayout[4][4] = {
        {0, 1, 2, 3},
        {4, 5, 6, 7},
        {8, 9, 10, 11},
        {12, 13, 14, 15}
    };
    static alert_level_t gridAlerts[16];

#ifdef useAudio
//static const char *TAG = "pwm_audio test";

// Some resources are lazy allocated in GPTimer driver, the threshold is left for that case
#define TEST_MEMORY_LEAK_THRESHOLD (-300)

#define LEFT_CHANNEL_GPIO  GPIO_NUM_13
#define RIGHT_CHANNEL_GPIO GPIO_NUM_5
#endif
 

#ifdef useRGBLed
SemaphoreHandle_t led_strip_mutex;
#endif
//SemaphoreHandle_t g_i2c_bus_mutex;
SemaphoreHandle_t uart_mutex;
SemaphoreHandle_t  ultrasound_mutex;
// ----------------------------------------------------------------
// --- NEW: Function Prototypes / Forward Declarations ---
// ----------------------------------------------------------------

// Low-level LED helper functions
#ifdef useRGBLed
void rgb_led_init();
void rgb_led_off(); // <-- This is the missing declaration
void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b);
void rgb_led_set_color_with_brightness(uint8_t r, uint8_t g, uint8_t b, float brightness);
uint8_t gamma_correct(uint8_t val);
#endif

// Beeper helper functions
esp_err_t init_piezo_beeper(void);
void set_beeper_tone(uint32_t frequency_hz, bool enable);

// Alert logic functions
alert_level_t get_alert_level(uint16_t distance_cm, uint16_t measured_average_cm);
void update_beeper_alerts(alert_level_t new_level, int direction);

// Task functions
extern "C" void read_and_parse_nmea();
//extern "C" void nmea_example_init_interface(void);

#ifdef useRGBLed
void led_control_task(void *pvParameters);
#endif
#ifdef useAccelerometer
void mpu9250_reader_task(void *pvParameters);
void data_processor_task(void *pvParameters);
#endif

#ifdef useAudio
//void audioTask(void *pvParameters);
//bool play_sine_tone(uint32_t duration_ms, uint32_t frequency, float amplitude) ;
void audioTask(void *parameter);
#endif


void vl53l5cx_reader_task(void *pvParameters);
bool vl53l5cx_recover(VL53L5CX_Configuration *dev);


#ifdef useLuna

/*
// Configuration
#define TF_LUNA_ADDR 0x10
#define TF_LUNA_CMD_LEN 5
#define TF_LUNA_DATA_LEN 9
#define TF_LUNA_TASK_STACK 3048
#define TF_LUNA_TASK_PRIO 5  // Medium priority
#define TF_LUNA_READ_DELAY_MS 200
// I2C error: 0x103
static const uint8_t tf_luna_command[TF_LUNA_CMD_LEN] = {0x5A, 0x05, 0x00, 0x01, 0x60};

// Data structure for sharing readings
typedef struct {
    uint16_t distance;
    uint16_t strength;
    float temperature;
} tf_luna_data_t;

// Global resources
static QueueHandle_t tf_luna_queue = NULL;
static i2c_master_dev_handle_t tf_luna_handle;
//extern i2c_master_bus_handle_t bus_handle;  // Your existing bus handle
//extern SemaphoreHandle_t g_i2c_bus_mutex;   // Your existing mutex
*/
// TF-Luna task
// Configuration - ADJUST THESE!
/*
#define I2C_PORT I2C_NUM_1
#define I2C_SDA_GPIO GPIO_NUM_1
#define I2C_SCL_GPIO GPIO_NUM_2
#define TF_LUNA_ADDR 0x10
#define TF_LUNA_CMD_LEN 5
#define TF_LUNA_DATA_LEN 9
#define TF_LUNA_TASK_STACK 1024 * 4
#define TF_LUNA_TASK_PRIO 5
#define TF_LUNA_READ_DELAY_MS 200
*/

// Configuration
#define SLAVE_ADDRESS 0x10
#define DATA_LENGTH 9
#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000
#define COMMAND_BUFFER {0x5A, 0x05, 0x00, 0x01, 0x60}

typedef struct {
    uint16_t distance;
    uint16_t strength;
    float temperature;
} sensor_data_t;

   static i2c_master_dev_handle_t tf_luna_handle = NULL;

static void initialize_i2c(void) {

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        }
    };



 //  i2c_port_t i2c_port = I2C_NUM_1;

 /* 
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags {
            .enable_internal_pullup = false,
            .allow_pd = false,
        }
    };

 // i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

       // 3. Add TF-Luna device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SLAVE_ADDRESS,
        .scl_speed_hz = 50000, // 100kHz standard speed
    };
    
 
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &tf_luna_handle);
*/
   ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
   ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static esp_err_t read_luna_data(sensor_data_t *data) {
    const uint8_t command[] = COMMAND_BUFFER;
    uint8_t response[DATA_LENGTH] = {0};

    // Send command
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, SLAVE_ADDRESS, 
        command, sizeof(command), pdMS_TO_TICKS(1000));
        ESP_LOGE(TAG, "I2C write: %d", ret);
    if (ret != ESP_OK) return ret;
   // Read response
    ret = i2c_master_read_from_device(I2C_MASTER_NUM, SLAVE_ADDRESS, 
        response, DATA_LENGTH, pdMS_TO_TICKS(1000));
         ESP_LOGE(TAG, "I2C read: %d", ret);
    if (ret != ESP_OK) return ret;

    // new driver
    /*
 esp_err_t ret = i2c_master_transmit(tf_luna_handle, command, sizeof(command), 1000);


ret = i2c_master_receive(tf_luna_handle, response, DATA_LENGTH, 1000);
 */


 
    // Parse data
    data->distance = response[2] + (response[3] << 8);
    data->strength = response[4] + (response[5] << 8);
    data->temperature = (response[6] + (response[7] << 8)) / 8.0f - 256.0f;
    
    return ESP_OK;
}

static void i2c_task(void *pvParameters) {
    ESP_LOGI(TAG, "I2C task started");
    sensor_data_t data;

    while (1) {
        esp_err_t ret = read_luna_data(&data);
        
        if (ret == ESP_OK) {
            // Send data to logging task via queue (or print directly)
            printf("Distance: %d cm, Strength: %d, Temperature: %.1f°C\n",
                  data.distance, data.strength, data.temperature);
        } else {
            ESP_LOGE(TAG, "I2C communication failed: %d", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms delay
    }
}

/*
static const uint8_t tf_luna_command[TF_LUNA_CMD_LEN] = {0x5A, 0x05, 0x00, 0x01, 0x60};

typedef struct {
    uint16_t distance;
    uint16_t strength;
    float temperature;
} tf_luna_data_t;

static QueueHandle_t tf_luna_queue = NULL;
//static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t tf_luna_handle = NULL;
//static SemaphoreHandle_t i2c_mutex = NULL;



#define MAX_RETRIES 3
esp_err_t tf_luna_read_with_retry(tf_luna_data_t *data) {
    uint8_t rx_data[TF_LUNA_DATA_LEN];
    esp_err_t ret;
    int retries = 0;

    while (retries < MAX_RETRIES) {
        ret = i2c_master_transmit(tf_luna_handle, tf_luna_command, TF_LUNA_CMD_LEN, 50);
        if (ret != ESP_OK) goto retry;

        ret = i2c_master_receive(tf_luna_handle, rx_data, TF_LUNA_DATA_LEN, 50);
        if (ret == ESP_OK) {
                         
                   uint32_t distance = rx_data[2] | (rx_data[3] << 8);
                   uint32_t strength = rx_data[4] | (rx_data[5] << 8);
                   uint32_t temperature = (rx_data[6] | (rx_data[7] << 8)) / 8.0f - 256.0f;
                    //xQueueSend(tf_luna_queue, &data, 0);

            ESP_LOGI("TF_LUNA", "retry: distance %d, strength %d, temperature %d", distance, strength, temperature);
               

                }
            return ESP_OK;
        

retry:
        retries++;
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGI("TF_LUNA", "retrying %d", retries);
    }
    return ret;
}
*/

/*
static void tf_luna_task(void *arg) {
    tf_luna_data_t data;
    uint8_t rx_data[TF_LUNA_DATA_LEN];

    while(1) {
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            esp_err_t ret = i2c_master_transmit(tf_luna_handle, tf_luna_command, TF_LUNA_CMD_LEN, pdMS_TO_TICKS(50));
            
            if (ret == ESP_OK) {
                ret = i2c_master_receive(tf_luna_handle, rx_data, TF_LUNA_DATA_LEN, pdMS_TO_TICKS(50));
                
                if (ret == ESP_OK) {
                    data.distance = rx_data[2] | (rx_data[3] << 8);
                    data.strength = rx_data[4] | (rx_data[5] << 8);
                    data.temperature = (rx_data[6] | (rx_data[7] << 8)) / 8.0f - 256.0f;
                    xQueueSend(tf_luna_queue, &data, 0);
                }
            }
            
            xSemaphoreGive(i2c_mutex);
            
            if (ret != ESP_OK) {
                ESP_LOGE("TF_LUNA", "I2C error: 0x%x (%s)", ret, esp_err_to_name(ret));
                vTaskDelay(pdMS_TO_TICKS(1000)); // Longer delay on error
                continue;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(TF_LUNA_READ_DELAY_MS));
    }
}
*/


/*
esp_err_t tf_luna_init() {
    // 1. Initialize I2C bus
    i2c_mutex = xSemaphoreCreateMutex();
    if (!i2c_mutex) return ESP_FAIL;


   i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags {
            .enable_internal_pullup = false,
            .allow_pd = false,
        }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    // 2. Create data queue
    tf_luna_queue = xQueueCreate(5, sizeof(tf_luna_data_t));
    if (!tf_luna_queue) {
        i2c_del_master_bus(bus_handle);
        return ESP_FAIL;
    }

    // 3. Add TF-Luna device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TF_LUNA_ADDR,
        .scl_speed_hz = 50000, // 100kHz standard speed
    };
    
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &tf_luna_handle);
    if (ret != ESP_OK) {
        vQueueDelete(tf_luna_queue);
     //   i2c_del_master_bus(bus_handle);
        return ret;
    } else {
 ESP_LOGI("TF_LUNA", "i2c_master_bus_add_device success");
    }

    // 4. Create task
    if (xTaskCreate(tf_luna_task, "tf_luna", TF_LUNA_TASK_STACK, NULL, TF_LUNA_TASK_PRIO, NULL) != pdPASS) {
        i2c_master_bus_rm_device(tf_luna_handle);
        vQueueDelete(tf_luna_queue);
      //  i2c_del_master_bus(bus_handle);
        return ESP_FAIL;
    }

    return ESP_OK;
}
*/

//#define TF_LUNA_ADDR          0x10
#define TF_LUNA_TIMEOUT_MS    50
#define TAG "TF-Luna"

// Register addresses (from Arduino library)
typedef enum {
    TFL_DIST_LO = 0x00,
    TFL_DIST_HI = 0x01,
    TFL_FLUX_LO = 0x02,
    TFL_FLUX_HI = 0x03,
    TFL_TEMP_LO = 0x04,
    TFL_TEMP_HI = 0x05,
    TFL_TICK_LO   =       0x06,  //R Timestamp
    TFL_TICK_HI   =        0x07,  //R
    TFL_ERR_LO   =         0x08,  //R
    TFL_ERR_HI   =         0x09,  //R
    TFL_VER_REV   =        0x0A,  //R
    TFL_VER_MIN  =         0x0B,  //R
    TFL_VER_MAJ  =         0x0C,  //R
} tfl_reg_t;

/*
typedef struct {
    i2c_master_dev_handle_t handle;
    uint16_t distance;
    uint16_t flux;
    int16_t temp;
} tfluna_t;

// Initialize TF-Luna
esp_err_t tfluna_init(i2c_master_bus_handle_t bus, tfluna_t* tfl) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TF_LUNA_ADDR,
        .scl_speed_hz = 100000,  // Standard 100kHz
    };
    return i2c_master_bus_add_device(bus, &dev_cfg, &tfl->handle);
}

// Read register (matches Arduino's readReg)
static esp_err_t tfluna_read_reg(tfluna_t* tfl, uint8_t reg, uint8_t* val) {
    uint8_t reg_addr = reg;
    esp_err_t ret = i2c_master_transmit_receive(
        tfl->handle,
        &reg_addr, 1,
        val, 1,
        pdMS_TO_TICKS(TF_LUNA_TIMEOUT_MS));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read reg 0x%02x failed: 0x%x", reg, ret);
    }
    return ret;
}

// Get data (matches Arduino's getData)
esp_err_t tfluna_get_data(tfluna_t* tfl) {
    uint8_t data[6];
    
    // Read 6 contiguous registers (distance, flux, temperature)
    for (uint8_t reg = TFL_DIST_LO; reg <= TFL_TEMP_HI; reg++) {
 //       ESP_RETURN_ON_ERROR(
          esp_err_t ret2 =  tfluna_read_reg(tfl, reg, &data[reg]);

             if (ret2 != ESP_OK) {
        ESP_LOGE(TAG, "Failed reading reg 0x%02x", reg);
    }

          //  TAG, "Failed reading reg 0x%02x", reg);
        
        vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between reads
    }

    // Parse values (little-endian)
    tfl->distance = data[0] | (data[1] << 8);
    tfl->flux = data[2] | (data[3] << 8);
    tfl->temp = (data[4] | (data[5] << 8)) / 8.0f - 256.0f;

    // Validate signal strength (as in Arduino lib)
    if (tfl->flux < 100) {
        ESP_LOGW(TAG, "Signal too weak: %d", tfl->flux);
        return ESP_ERR_INVALID_RESPONSE;
    } else if (tfl->flux == 0xFFFF) {
        ESP_LOGW(TAG, "Signal saturated");
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

// Example usage:
void tf_luna_task(void* arg) {
    i2c_master_bus_handle_t bus = (i2c_master_bus_handle_t)arg;
    tfluna_t tfl;
    
    ESP_ERROR_CHECK(tfluna_init(bus, &tfl));
    ESP_LOGI(TAG, "TF-Luna initialized");

    while(1) {
        esp_err_t ret = tfluna_get_data(&tfl);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Distance: %dcm, Flux: %d, Temp: %.1f°C", 
                    tfl.distance, tfl.flux, tfl.temp);
        } else {
            ESP_LOGW(TAG, "Measurement failed (ret: 0x%x)", ret);
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Adjust as needed
    }
}
*/







#endif

//void blink_task(void *pvParameters);
// Task prototype
//void blink_task_init(gpio_num_t pin, uint8_t times, uint8_t interval_ms);

#ifdef useRGBLed
void testblink(){
  led_strip_set_pixel(led_strip, 0, 16, 0, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);

          vTaskDelay(pdMS_TO_TICKS(300));
  led_strip_set_pixel(led_strip, 0, 0, 16, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
 vTaskDelay(pdMS_TO_TICKS(300));
          led_strip_set_pixel(led_strip, 0, 0, 0, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
  vTaskDelay(pdMS_TO_TICKS(300));

 led_strip_clear(led_strip);
}
#endif


/*
void watchdog_feed() {
    wdt_feed();
}
//esp_task_wdt_feed()

void init_watchdog() {
    wdt_enable( WDT_PERIPH_TIMEOUT_S( 60 )); // 60 seconds timeout
    xTaskCreate(&watchdog_feed_task, "wdt_feed", 2048, NULL, 1, NULL);
}

void watchdog_feed_task(void *pvParameters) {
    while(1) {
        watchdog_feed();
        vTaskDelay( pdMS_TO_TICKS( 50000 )); // Feed every 50 seconds
    }
}
*/


 #define I2C_SCAN_PORT I2C_NUM_1
#define I2C_SCAN_SDA_GPIO 1
#define I2C_SCAN_SCL_GPIO 2
#define I2C_SCAN_TIMEOUT_MS 100

//static i2c_master_bus_handle_t bus_handle = NULL;
//static SemaphoreHandle_t i2c_mutex = NULL;


// Initialize I2C bus if not already initialized
static esp_err_t init_i2c_bus() {
    if (bus_handle == NULL) {
        i2c_mutex = xSemaphoreCreateMutex();
        if (i2c_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2C mutex");
            return ESP_FAIL;
        }
/*
       i2c_master_bus_config_t i2c_bus_config = {
            .i2c_port = I2C_SCAN_PORT,
            .sda_io_num = I2C_SCAN_SDA_GPIO,
            .scl_io_num = I2C_SCAN_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
*/
    i2c_port_t i2c_port = I2C_NUM_1;

   i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = i2c_port,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags {
            .enable_internal_pullup = true,
            .allow_pd = false,
        }
    };

    //i2c_master_bus_handle_t bus_handle;
    //ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));


      //  g_i2c_bus_mutex

        esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
            vSemaphoreDelete(i2c_mutex);
            return ret;
        }
    }
    return ESP_OK;
}

// Scan I2C bus and print found devices
void scan_i2c_devices() {

    /*
        i2c_port_t i2c_port = I2C_NUM_1;

  i2c_master_bus_handle_t bus_handle;
 
   i2c_master_bus_config_t bus_cfg = {
        .i2c_port = i2c_port,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
      //  .glitch_ignore_cnt = 7,
        .flags {
            .enable_internal_pullup = false,
         //   .allow_pd = false,
        }
    };

 */
 /*
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .flags.enable_internal_pullup = false
    };
    */

  //  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        i2c_device_config_t dev_cfg = {
            .device_address = addr,
            .scl_speed_hz = 100000
        };
        i2c_master_dev_handle_t dev_handle;
        esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
        if (ret == ESP_OK) {
            uint8_t dummy;
            if (i2c_master_receive(dev_handle, &dummy, 1, 50) == ESP_OK) {
                ESP_LOGI("I2C_SCAN", "Found device at 0x%02X", addr);
            }
            i2c_master_bus_rm_device(dev_handle);
        } else {
             ESP_LOGI("I2C_SCAN", "Not OK");
        }
    }
    i2c_del_master_bus(bus_handle);
}


#ifdef useRGBLed
/**
 * @brief Helper function to print the contents of an led_state_t struct
 * 
 * @param tag The ESP_LOG tag to use.
 * @param prefix A string to print before the struct data.
 * @param state The struct to print.
 */
void log_led_state(const char* tag, const char* prefix, const led_state_t& state)
{
    ESP_LOGI(tag, "%s r=%u, g=%u, b=%u, bright=%.2f, top=%s, bot=%s",
             prefix,
             state.r,
             state.g,
             state.b,
             state.brightness,
             (state.top_led_on ? "ON" : "OFF"),
             (state.bottom_led_on ? "ON" : "OFF"));
}
// ----------------------------------------------------------------
// --- NEW: LED Control Task Definitions ---
// ----------------------------------------------------------------

// The handle for our command queue
//QueueHandle_t led_command_queue;

// ----------------------------------------------------------------
// --- FINAL, CORRECTED led_control_task (The "Renderer") ---
// ----------------------------------------------------------------
// -------------------------------------------------------------------------
// --- DEFINITIVE, CORRECTED led_control_task ---
// -------------------------------------------------------------------------
// ----------------------------------------------------------------
// --- FINAL, Rock-Solid led_control_task (The "Renderer") ---
// ----------------------------------------------------------------
// ----------------------------------------------------------------
// --- DEFINITIVE, Rock-Solid, Event-Driven led_control_task ---
// ----------------------------------------------------------------
void led_control_task(void *pvParameters)
{
    // A variable to hold the new state when it arrives from the queue.
    led_state_t new_state{}; 

    ESP_LOGI("LED_TASK", "Event-Driven Renderer Task Started. Waiting for state updates...");

    // This is the entire task.
    while (1)
    {
        // Block and wait INDEFINITELY until a new state is received from the queue.
        // This is extremely efficient; the task uses 0% CPU while waiting.
        if (xQueueReceive(led_state_queue, &new_state, portMAX_DELAY) == pdPASS) 
        {
            // --- A new state has arrived. Render it ONCE. ---
            
            ESP_LOGI("LED_TASK", "New state received. Updating hardware.");
            log_led_state("LED_TASK", "New State:", new_state); // Using the helper function to log

            // Render GPIO LEDs based on the new state
            gpio_set_level(RED_PIN, new_state.top_led_on);
            gpio_set_level(BLUE_PIN, new_state.bottom_led_on);

            // Render WS2812 LED based on the new state
            if (new_state.brightness > 0.0f) {
                ESP_LOGI("LED_TASK", "brightness > 0");
                rgb_led_set_color_with_brightness(new_state.r, new_state.g, new_state.b, new_state.brightness);
            } else {
                ESP_LOGI("LED_TASK", "brightness < 0");
                rgb_led_off();
            }

            // After rendering, the task loops back and immediately sleeps again,
            // waiting for the next message. There is no vTaskDelay here.
        }
    }
}

void led_control_task_old(void *pvParameters)
{
    // This variable holds the last known "desired state" received from the queue.
    led_state_t current_state{}; // Starts with all LEDs off

    ESP_LOGI("LED_TASK", "Renderer Task Started.");

    while (1)
    {
        // Check if a new desired state has been sent. This is non-blocking.
        xQueueReceive(led_state_queue, &current_state, 0);

        // --- Render the GPIO LEDs state ---
        gpio_set_level(RED_PIN, current_state.top_led_on);
        gpio_set_level(BLUE_PIN, current_state.bottom_led_on);

        // --- Render the WS2812 LED state ---
        // If brightness is 0, we can turn it off. Otherwise, set the color.
        if (current_state.brightness > 0.0f) {

            ESP_LOGI("LED_TASK", "current_state.brightness > 0");
            rgb_led_set_color_with_brightness(current_state.r, current_state.g, current_state.b, current_state.brightness);
        } else {
            rgb_led_off();
        }
        
        // Loop at a reasonable "refresh rate" for the LEDs.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
#endif

#ifdef useGPS_2

#define UART_RX_BUF_SIZE 1024
#define UART_RX_PIN 17

static char s_buf[UART_RX_BUF_SIZE + 1];
static size_t s_total_bytes;
static char *s_last_buf_end;

extern "C" void nmea_example_init_interface(void)
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM,
                                 UART_PIN_NO_CHANGE, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
}

void nmea_example_read_line(char **out_line_buf, size_t *out_line_len, int timeout_ms)
{
    *out_line_buf = NULL;
    *out_line_len = 0;

    if (s_last_buf_end != NULL) {
        /* Data left at the end of the buffer after the last call;
         * copy it to the beginning.
         */
        size_t len_remaining = s_total_bytes - (s_last_buf_end - s_buf);
        memmove(s_buf, s_last_buf_end, len_remaining);
        s_last_buf_end = NULL;
        s_total_bytes = len_remaining;
    }

    /* Read data from the UART */
    int read_bytes = uart_read_bytes(UART_NUM,
                                     (uint8_t *) s_buf + s_total_bytes,
                                     UART_RX_BUF_SIZE - s_total_bytes, pdMS_TO_TICKS(timeout_ms));
    if (read_bytes <= 0) {
        return;
    }
    s_total_bytes += read_bytes;

    /* find start (a dollar sign) */
    char *start = memchr(s_buf, '$', s_total_bytes);
    if (start == NULL) {
        s_total_bytes = 0;
        return;
    }

    /* find end of line */
    char *end = memchr(start, '\r', s_total_bytes - (start - s_buf));
    if (end == NULL || *(++end) != '\n') {
        return;
    }
    end++;

    end[-2] = NMEA_END_CHAR_1;
    end[-1] = NMEA_END_CHAR_2;

    *out_line_buf = start;
    *out_line_len = end - start;
    if (end < s_buf + s_total_bytes) {
        /* some data left at the end of the buffer, record its position until the next call */
        s_last_buf_end = end;
    } else {
        s_total_bytes = 0;
    }
}


extern "C" void read_and_parse_nmea()
{
    while (1) {
        char fmt_buf[32];
        nmea_s *data;

        char *start;
        size_t length;
        nmea_example_read_line(&start, &length, 100 /* ms */);
        if (length == 0) {
            continue;
        }

        /* handle data */
        data = nmea_parse(start, length, 0);
        if (data == NULL) {
            printf("Failed to parse the sentence!\n");
            printf("  Type: %.5s (%d)\n", start + 1, nmea_get_type(start));
        } else {
            if (data->errors != 0) {
                printf("WARN: The sentence struct contains parse errors!\n");
            }

            if (NMEA_GPGGA == data->type) {
                printf("GPGGA sentence\n");
                nmea_gpgga_s *gpgga = (nmea_gpgga_s *) data;
                printf("Number of satellites: %d\n", gpgga->n_satellites);
                printf("Altitude: %f %c\n", gpgga->altitude,
                       gpgga->altitude_unit);
            }

            if (NMEA_GPGLL == data->type) {
                printf("GPGLL sentence\n");
                nmea_gpgll_s *pos = (nmea_gpgll_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%H:%M:%S", &pos->time);
                printf("Time: %s\n", fmt_buf);
            }

            if (NMEA_GPRMC == data->type) {
                printf("GPRMC sentence\n");
                nmea_gprmc_s *pos = (nmea_gprmc_s *) data;
                printf("Longitude:\n");
                printf("  Degrees: %d\n", pos->longitude.degrees);
                printf("  Minutes: %f\n", pos->longitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->longitude.cardinal);
                printf("Latitude:\n");
                printf("  Degrees: %d\n", pos->latitude.degrees);
                printf("  Minutes: %f\n", pos->latitude.minutes);
                printf("  Cardinal: %c\n", (char) pos->latitude.cardinal);
                strftime(fmt_buf, sizeof(fmt_buf), "%d %b %T %Y", &pos->date_time);
                printf("Date & Time: %s\n", fmt_buf);
                printf("Speed, in Knots: %f\n", pos->gndspd_knots);
                printf("Track, in degrees: %f\n", pos->track_deg);
                printf("Magnetic Variation:\n");
                printf("  Degrees: %f\n", pos->magvar_deg);
                printf("  Cardinal: %c\n", (char) pos->magvar_cardinal);
                double adjusted_course = pos->track_deg;
                if (NMEA_CARDINAL_DIR_EAST == pos->magvar_cardinal) {
                    adjusted_course -= pos->magvar_deg;
                } else if (NMEA_CARDINAL_DIR_WEST == pos->magvar_cardinal) {
                    adjusted_course += pos->magvar_deg;
                } else {
                    printf("Invalid Magnetic Variation Direction!\n");
                }

                printf("Adjusted Track (heading): %f\n", adjusted_course);
            }

            if (NMEA_GPGSA == data->type) {
                nmea_gpgsa_s *gpgsa = (nmea_gpgsa_s *) data;

                printf("GPGSA Sentence:\n");
                printf("  Mode: %c\n", gpgsa->mode);
                printf("  Fix:  %d\n", gpgsa->fixtype);
                printf("  PDOP: %.2lf\n", gpgsa->pdop);
                printf("  HDOP: %.2lf\n", gpgsa->hdop);
                printf("  VDOP: %.2lf\n", gpgsa->vdop);
            }

            if (NMEA_GPGSV == data->type) {
                nmea_gpgsv_s *gpgsv = (nmea_gpgsv_s *) data;

                printf("GPGSV Sentence:\n");
                printf("  Num: %d\n", gpgsv->sentences);
                printf("  ID:  %d\n", gpgsv->sentence_number);
                printf("  SV:  %d\n", gpgsv->satellites);
                printf("  #1:  %d %d %d %d\n", gpgsv->sat[0].prn, gpgsv->sat[0].elevation, gpgsv->sat[0].azimuth, gpgsv->sat[0].snr);
                printf("  #2:  %d %d %d %d\n", gpgsv->sat[1].prn, gpgsv->sat[1].elevation, gpgsv->sat[1].azimuth, gpgsv->sat[1].snr);
                printf("  #3:  %d %d %d %d\n", gpgsv->sat[2].prn, gpgsv->sat[2].elevation, gpgsv->sat[2].azimuth, gpgsv->sat[2].snr);
                printf("  #4:  %d %d %d %d\n", gpgsv->sat[3].prn, gpgsv->sat[3].elevation, gpgsv->sat[3].azimuth, gpgsv->sat[3].snr);
            }

            if (NMEA_GPTXT == data->type) {
                nmea_gptxt_s *gptxt = (nmea_gptxt_s *) data;

                printf("GPTXT Sentence:\n");
                printf("  ID: %d %d %d\n", gptxt->id_00, gptxt->id_01, gptxt->id_02);
                printf("  %s\n", gptxt->text);
            }

            if (NMEA_GPVTG == data->type) {
                nmea_gpvtg_s *gpvtg = (nmea_gpvtg_s *) data;

                printf("GPVTG Sentence:\n");
                printf("  Track [deg]:   %.2lf\n", gpvtg->track_deg);
                printf("  Speed [kmph]:  %.2lf\n", gpvtg->gndspd_kmph);
                printf("  Speed [knots]: %.2lf\n", gpvtg->gndspd_knots);
            }

            nmea_free(data);
        }

vTaskDelay(pdMS_TO_TICKS(1000));

    }
}
#endif


#ifdef useGPS
/*
typedef struct
{
    double latitude;
    double longitude;
    double speed_kmh; // 单位：千米每小时
    double speed_ms;  // 单位：米每秒
} GPS_data;
*/


void gps_task(void *arg)
{

//ESP_ERROR_CHECK(esp_task_wdt_add(NULL));



for (;;) {
   
esp_task_wdt_delete(NULL); 
//    ESP_ERROR_CHECK(esp_task_wdt_reset());

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal for GPS reading loop.");


ESP_LOGI("GPS", "GPS starting");

ESP_LOGD("GPS", "GPS Waiting for uart_mutex ...");
if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

//_hasMutex = true;
vTaskDelay(pdMS_TO_TICKS(100));


   // const char *TAG = "GPS";
    while (1)
    {
 //esp_task_wdt_delete(NULL); 
       //  ESP_ERROR_CHECK(esp_task_wdt_reset());

        GPS_data gps_data = gps_get_value();
        ESP_LOGI("GPS", "lat:%f, lon:%f alt:%f", gps_data.latitude, gps_data.longitude, gps_data.altitude); 
        ESP_LOGI("GPS", "speed:%f", gps_data.speed_ms); 
        ESP_LOGI("GPS", "course:%f", gps_data.course);  
        ESP_LOGI("GPS", "Date:%d/%d/%d Time:%d:%d:%d",gps_data.day, gps_data.month, gps_data.year, gps_data.hour, gps_data.minute,gps_data.second); 
// Date:25/6/28 Time:15:34:42
       // ESP_LOGI(TAG, "======================================================\n");
         vTaskDelay(2000);
       // vTaskDelay(1000 / portTICK_RATE_MS);

                      // --- CRITICAL PART: Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal for GPS loop.");
                    break; // Exit the inner "running" loop
                }
//esp_task_wdt_add(NULL);
    }

    xSemaphoreGive(uart_mutex);
ESP_LOGD("GPS", "GPS Giving uart_mutex ...");
}  // got mutex

} // startbit
vTaskDelay(pdMS_TO_TICKS(100));

esp_task_wdt_add(NULL);
} // outer loop wait for startbit

}
#endif

// Task parameters structure (for potential future expansion)
typedef struct {
    gpio_num_t pin;
    uint8_t times;
    uint32_t interval_ms;
} blink_task_params_t;

// Static storage for task parameters
static blink_task_params_t blink_task_params;

// **Embedded Blink Task Function**
static void blink_task(void *pvParameter) {

  //  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));


 //    if (xSemaphoreTake(audio_mutex, portMAX_DELAY) == pdTRUE) {
//ESP_LOGD(TAG, "blink audio Got mutex .");



    // Configure GPIO pin as output
 //   gpio_pad_select_gpio(BLINK_PIN);
    gpio_set_direction(blink_task_params.pin, GPIO_MODE_OUTPUT);

//ESP_LOGI("BLINK", "blink_task %d %d %d", blink_task_params.pin,blink_task_params.times, blink_task_params.interval_ms );



    // Blink 'times' times with 'interval_ms' between blinks
    for (uint8_t i = 0; i < blink_task_params.times; i++) {
      //  ESP_ERROR_CHECK(esp_task_wdt_reset());
        gpio_set_level(blink_task_params.pin, 1);
        vTaskDelay(pdMS_TO_TICKS(blink_task_params.interval_ms / 2));
        gpio_set_level(blink_task_params.pin, 0);
        vTaskDelay(pdMS_TO_TICKS(blink_task_params.interval_ms - (blink_task_params.interval_ms / 2)));
    }
    gpio_set_level(blink_task_params.pin, 0); // Final state
    vTaskDelay(pdMS_TO_TICKS(1));


 //  xSemaphoreGive(audio_mutex);

//ESP_LOGD(TAG, "blink audio gave mutex .");
vTaskDelete(NULL);

//} else {
//ESP_LOGE(TAG, "blink audio no mutex .");

//}

}

// **Embedded Function to Initialize Blink Task**
static void blink_task_init( gpio_num_t pin, uint8_t times,  uint32_t interval_ms) {
    blink_task_params.pin = pin;
    blink_task_params.times = times;

    blink_task_params.interval_ms = interval_ms;
TaskHandle_t xHandle_blink = NULL;
//  vTaskDelete( xHandle_blink );

 //     if (xHandle_blink = NULL){
//ESP_LOGI("BLINK", "blink_task_init %d %d %d", pin,times, interval_ms );

    xTaskCreate(blink_task, "BlinkTask", 2048, NULL, 2, &xHandle_blink);
 //     }

}

#ifdef useAudio
static void play_with_param(ledc_timer_bit_t duty_resolution, uint8_t hw_ch);

static void play_audio(const uint8_t *data, size_t wave_size, uint32_t rate, ledc_timer_bit_t bits, uint32_t ch);

bool play_sine_tone(uint32_t duration_ms, uint32_t frequency, float amplitude);

extern const uint8_t wave_array_32000_8_1[];
extern const uint8_t wave_array_32000_8_2[];
extern const uint8_t wave_array_32000_16_1[];
extern const uint8_t wave_array_32000_16_2[];

extern const int16_t aurora[];
extern const int16_t aurora_slow[];
extern const int16_t aurora_fast[];
#include "aurora.h" // This file was generated by wav2c
#include "aurora_slow.h" // This file was generated by wav2c
#include "aurora_fast.h" // This file was generated by wav2c

void play_wav_from_flash(alert_level_t alert_level ) {
    ESP_LOGI(TAG, "Playing audio wav from flash memory...");

 // ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
         


    /*
   pwm_audio_config_t pac;
    pac.duty_resolution    = LEDC_TIMER_10_BIT;
    pac.gpio_num_left      = LEFT_CHANNEL_GPIO;
    pac.ledc_channel_left  = LEDC_CHANNEL_0;
   // if (2 == hw_ch) {
     //   pac.gpio_num_right     = RIGHT_CHANNEL_GPIO;
   // } else {
        pac.gpio_num_right     = -1;
   // }
    pac.ledc_channel_right = LEDC_CHANNEL_1;
    pac.ledc_timer_sel     = LEDC_TIMER_0;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    pac.tg_num             = TIMER_GROUP_0;
    pac.timer_num          = TIMER_0;
#endif
    pac.ringbuf_len        = 1024 * 8;

pwm_audio_init(&pac);

pwm_audio_set_param(32000, (ledc_timer_bit_t) 16, 1);

  pwm_audio_start();
*/
  switch (alert_level) {
        case 7:   pwm_audio_set_volume(10); break;
        case 6:   pwm_audio_set_volume(10); break;
          case 5:    pwm_audio_set_volume(8); break;
          case 4: pwm_audio_set_volume(6); break;
          case 3: pwm_audio_set_volume(4); break;
          case 2:   pwm_audio_set_volume(1); break;
          case 1: pwm_audio_set_volume(1); break;
          case 0:   break;
          default:            break;
        }


   // Select which audio sample to play
  //  const int16_t* audio_data;
 //   size_t array_size_bytes;
 
    // Pitch adjustment factors
    float pitch_factor = 1.0f; // Default normal speed
    switch(alert_level) {
         case 7: pitch_factor = 1.5f; break;
        case 6: pitch_factor = 1.25f; break; // Faster/higher pitch for immediate alerts
        case 5: pitch_factor = 1.0f; break; // Slower/lower pitch for close alerts
        case 4: pitch_factor = 0.75f; break; 
        default: pitch_factor = 0.5f; break;
    }


  //  #define SAMPLE_RATE_MAX     (48000)
//#define SAMPLE_RATE_MIN     (8000)


    // Use the original aurora sample
    const size_t original_samples = sizeof(aurora_slow) / sizeof(int16_t);
    const uint32_t original_sample_rate = 32000;
    const uint32_t target_sample_rate = original_sample_rate * pitch_factor;


    // Set the output sample rate (this affects pitch)
    pwm_audio_set_param(target_sample_rate, (ledc_timer_bit_t)16, 1);
    pwm_audio_start();

    // Write the original audio data (will play at modified rate)
    size_t bytes_written;
    pwm_audio_write((uint8_t*)aurora_slow, sizeof(aurora_slow), &bytes_written, portMAX_DELAY);

    // Calculate actual duration (original duration / pitch factor)
    float original_duration = (original_samples * 1000.0f) / original_sample_rate;
    uint32_t actual_duration = original_duration / pitch_factor;
    
    
    /*

       
    switch (alert_level) {
        case 6:  // ALERT_IMMEDIATE
            audio_data = aurora_fast;
            array_size_bytes = sizeof(aurora_fast);
            break;
        case 5:  // ALERT_CLOSE
            audio_data = aurora_slow;
            array_size_bytes = sizeof(aurora_slow);
            break;
        default:
            audio_data = aurora_slow;
            array_size_bytes = sizeof(aurora_slow);
            break;
    }



   // Write the data to the player
    size_t bytes_written;
    pwm_audio_write((uint8_t*)audio_data, array_size_bytes, &bytes_written, portMAX_DELAY);

    // Calculate duration to wait for playback to finish
    int num_samples = array_size_bytes / sizeof(int16_t);
    uint32_t duration_ms = (num_samples * 1000) / 32000;  // Using actual sample rate of 32000
    ESP_LOGI(TAG, "WAV data written. Waiting %lu ms for playback.", duration_ms);
   // vTaskDelay(pdMS_TO_TICKS(duration_ms + 50)); // Wait for it to play + grace period
*/
    // Clean up

 vTaskDelay(pdMS_TO_TICKS(actual_duration + 5 )); // Wait for it to play + grace period

    pwm_audio_stop();
    ESP_LOGI(TAG, "Playback finished.");
}


static void play_audio(const uint8_t *data, size_t wave_size, uint32_t rate, ledc_timer_bit_t bits, uint32_t ch)
{

 //     ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
       



    uint32_t index = 0;
    size_t cnt;
    uint32_t block_w = 2048;
    ESP_LOGI(TAG, "parameter: samplerate:%"PRIu32", bits:%"PRIu32", channel:%"PRIu32"", rate, bits, ch);
    pwm_audio_set_param(rate, bits, ch);
    pwm_audio_start();

    while (1) {
        if (index < wave_size) {
            if ((wave_size - index) < block_w) {
                block_w = wave_size - index;
            }
            pwm_audio_write((uint8_t*)data + index, block_w, &cnt, 1000 / portTICK_PERIOD_MS);
            ESP_LOGD(TAG, "write [%"PRIu32"] [%d]", block_w, cnt);
            index += cnt;
        } else {
            ESP_LOGI(TAG, "play completed");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);


           ESP_ERROR_CHECK(esp_task_wdt_reset());


    }
    pwm_audio_stop();
}

static void play_with_param(ledc_timer_bit_t duty_resolution, uint8_t hw_ch)
{
    pwm_audio_config_t pac;
    pac.duty_resolution    = duty_resolution;
    pac.gpio_num_left      = LEFT_CHANNEL_GPIO;
    pac.ledc_channel_left  = LEDC_CHANNEL_0;
    if (2 == hw_ch) {
        pac.gpio_num_right     = RIGHT_CHANNEL_GPIO;
    } else {
        pac.gpio_num_right     = -1;
    }
    pac.ledc_channel_right = LEDC_CHANNEL_1;
    pac.ledc_timer_sel     = LEDC_TIMER_0;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    pac.tg_num             = TIMER_GROUP_0;
    pac.timer_num          = TIMER_0;
#endif
    pac.ringbuf_len        = 1024 * 8;

pwm_audio_init(&pac);

    ESP_LOGI(TAG, "\n------------------------------");
    ESP_LOGI(TAG, "hw info: resolution: %dbit, ch: %d", duty_resolution, hw_ch);
 //   pwm_audio_set_volume(8);
 //   play_audio(wave_array_32000_8_1, 64000, 32000, LEDC_TIMER_8_BIT, 1);
  //  pwm_audio_set_volume(8);
 //   play_audio(wave_array_32000_8_2, 128000, 32000, LEDC_TIMER_8_BIT, 2);
    pwm_audio_set_volume(8);
    ESP_LOGI(TAG, "vol 8");
    play_audio(wave_array_32000_16_1, 128000, 32000, (ledc_timer_bit_t) 16, 1);
    play_audio(wave_array_32000_16_1, 128000, 32000, (ledc_timer_bit_t) 16, 2);
    pwm_audio_set_volume(15);
    ESP_LOGI(TAG, "vol 15");
      play_audio(wave_array_32000_16_2, 256000, 32000, (ledc_timer_bit_t) 16, 1);
    play_audio(wave_array_32000_16_2, 256000, 32000, (ledc_timer_bit_t) 16, 2);

pwm_audio_deinit();

    ESP_LOGI(TAG, "------------------------------\n");
}


/**
 * @brief Generates and plays a sine wave tone for a given frequency and duration.
 *
 * This function is adapted for ESP-IDF. It dynamically allocates memory, fills it with
 * sine wave data, plays it using pwm_audio, and then frees the memory.
 *

 * @param duration_ms The duration of the tone in milliseconds.
 * @param frequency The frequency of the tone in Hz.
 */
bool play_sine_tone(uint32_t duration_ms, uint32_t frequency, float amplitude) {

//play_wav_from_flash();

 //play_with_param(LEDC_TIMER_8_BIT, 1);
 //   play_with_param(LEDC_TIMER_10_BIT, 1);
 // //  play_with_param(LEDC_TIMER_13_BIT, 1);

  //   play_with_param(LEDC_TIMER_8_BIT, 2);
  //  play_with_param(LEDC_TIMER_10_BIT, 2);
 ////   play_with_param(LEDC_TIMER_13_BIT, 2);

//return true;

uint32_t sample_rate_2 = 32000;

       // --- Input Validation ---
    // Clamp amplitude to the valid range [0.0, 1.0] to prevent distortion/wrap-around.
    if (amplitude > 1.0f) {
        amplitude = 1.0f;
    }
    if (amplitude < 0.0f) {
        amplitude = 0.0f;
    }

    if (frequency == 0) {
        // A frequency of 0 is silence, we can just delay.
      //  vTaskDelay(pdMS_TO_TICKS(duration_ms));
        return false;
    }


if (xSemaphoreTake(audio_mutex, portMAX_DELAY) == pdTRUE) {
 ESP_LOGD(TAG, "audio Got mutex .");


/*
if (ultraSensorDistance1 > 5 && ultraSensorDistance1 < 200){
double slope = 1.0 * (400 - 1000) / (300 - 5);
frequency = 400 + slope * (ultraSensorDistance1 - 5);
}
*/

//output = (output_end - output_start)/(input_end - input_start) *( input - input_start) + output_start;
//if (ultraSensorDistance1 > 5 && ultraSensorDistance1 < 300){
//int frequency = (1000 - 400)/(300 - 5) *( ultraSensorDistance1 - 5) + 400;
//}
 // Distance from sensors 1:5 2:193
 //frequency = ultraSensorDistance1
 //ultraSensorDistance2
    ESP_LOGI(TAG, "Playing tone: %lu Hz, %lu ms, Amplitude: %.2f", frequency, duration_ms, amplitude);

  //  ESP_LOGI(TAG, "play_sine_tone tone: %lu Hz, %lu ms", frequency, duration_ms);

    // 1. Calculate the total number of samples needed
    uint32_t total_samples = (sample_rate_2 * duration_ms) / 1000;

    // 2. Allocate memory for the audio buffer on the heap
    // We use uint8_t because we specified an 8-bit depth
    uint8_t *sine_wave_buffer = (uint8_t *)malloc(total_samples * sizeof(uint8_t));

    // IMPORTANT: Check if memory allocation was successful
    if (sine_wave_buffer == nullptr) {
        ESP_LOGE(TAG, "Error: Failed to allocate memory for audio buffer!");
        return false;
    }

        // 3. Generate the sine wave samples with volume control
    // The maximum amplitude for an 8-bit signal is 127.5. We scale this by our parameter.
//int volume = 15 - amplitude * 15;
int volume =  amplitude * 12;

pwm_audio_set_volume(volume);

ESP_LOGI(TAG, "Volume: %d ", volume);

    float max_amplitude = 127.5f * amplitude;

        for (uint32_t i = 0; i < total_samples; ++i) {
        double angle = 2.0 * M_PI * frequency * i / SAMPLE_RATE;
        // The oscillating part is now scaled by our desired amplitude
     //   sine_wave_buffer[i] = (uint8_t)(127.5 + max_amplitude * sin(angle));

         sine_wave_buffer[i] = (uint8_t)(127.5 + 127.5 * sin(angle));

    }


/*
int8_t volume = 5; // amplitude * 16;
pwm_audio_set_volume(volume);
//- 16 to 16
 
//if (i % 50 == 0){
//    volume--;
   // pwm_audio_set_volume(volume);
//}
int divider = total_samples/100;
int halfway = total_samples/4;
divider = halfway/10;


    // 3. Generate the sine wave samples
    for (uint32_t i = 0; i < total_samples; ++i) {
        // The formula for a sine wave is: Amplitude * sin(2 * PI * frequency * time)
        // 'time' is the sample number 'i' divided by the sample rate.
        // Amplitude for 8-bit audio is 127.5. We add 127.5 to shift the
        // wave (-1 to +1) to the unsigned 8-bit range (0 to 255).
        double angle = 2.0 * M_PI * frequency * i / SAMPLE_RATE;
    ////     ESP_LOGI(TAG, "Sample: %d of %d", i, total_samples);
// 1600 samples

if (i % (divider) == 0 && i > halfway && volume > 1 ){
    volume--;
 //   pwm_audio_set_volume(volume);
     ESP_LOGI(TAG, "- volume at %d: %d ", i, volume);
}

if (i % (divider) == 0 && i < halfway && volume < 15){
    volume++;
   // pwm_audio_set_volume(volume);
     ESP_LOGI(TAG, "+ volume at %d: %d ", i, volume);
}

 pwm_audio_set_volume(volume);
 */
/*
  for (int i = 20; i >= 0; --i) {
            float current_amplitude = i / 20.0f; // Ramps from 1.0 down to 0.0
            play_sine_tone(600, 50, current_amplitude);
        }


      sine_wave_buffer[i] = (uint8_t)(127.5 + 127.5 * sin(angle));
    }
*/

    // 4. Configure and start the PWM audio player
 //   pwm_audio_config_t config = {
 //       .duty_resolution = LEDC_TIMER_8_BIT, // Matches our 8-bit data
 //       .timer_num = LEDC_TIMER_0,
 //       .ringbuf_len = 1024 * 8,             // Ring buffer size, adjust as needed
 //   };

   
 //   pwm_audio_start(LEFT_CHANNEL_GPIO, &config);

    // 5. Write the generated audio data to the player's ring buffer
    size_t bytes_written;
    // 'portMAX_DELAY' will block until all data is written to the buffer
    pwm_audio_write(sine_wave_buffer, total_samples, &bytes_written, portMAX_DELAY);

    // If not all bytes could be written, it indicates an issue (e.g., buffer too small)
    if (bytes_written < total_samples) {
        ESP_LOGE(TAG, "Not all bytes written to buffer. Wrote %d of %lu", bytes_written, total_samples);
    }
    
    // 6. Wait for the sound to finish playing
    // The library plays in the background, so we must manually wait here.
    // Add a small grace period to ensure the buffer is fully played.
    vTaskDelay(pdMS_TO_TICKS(duration_ms + 20));

    // 7. Stop the player and clean up resources
   //pwm_audio_stop();

    // 8. CRITICAL: Free the allocated memory to prevent memory leaks
    free(sine_wave_buffer);

    ESP_LOGI(TAG, "Playback finished.");



xSemaphoreGive(audio_mutex);
ESP_LOGD(TAG, "audio gave mutex .");

   
} // got mutex

 else {
ESP_LOGD(TAG, "no audio  mutex .");

}

 return true;

}


bool initPWMaudio2 (){

  //  return true;
    
    // pac.ringbuf_len        = 1024 * 8;
  //  return true;
    // Configure PWM audio
        pwm_audio_config_t pac = {
        .gpio_num_left = LEFT_CHANNEL_GPIO,
        .gpio_num_right = RIGHT_CHANNEL_GPIO,
        .ledc_channel_left = LEDC_CHANNEL_0,
        .ledc_channel_right = LEDC_CHANNEL_1,
        .ledc_timer_sel = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .ringbuf_len = 1024 * 8
    };

    // Initialize audio system
    if (pwm_audio_init(&pac) != ESP_OK) {
       return false;
      ESP_LOGW(TAG, "pwm_audio_init error");
    }


  //  pwm_audio_set_param(32000, (ledc_timer_bit_t) 10, 1);

pwm_audio_set_param(32000, (ledc_timer_bit_t) 16, 1);

  pwm_audio_start();
  return true;
}

bool playAudio(uint32_t duration_ms, uint32_t frequency_hz) {
    // Validate input parameters
    if (frequency_hz == 0 || duration_ms == 0) {
        return false;
    }

 // ESP_ERROR_CHECK(esp_task_wdt_add(NULL));


ESP_LOGI(TAG, "playAudio %d %d", duration_ms, frequency_hz);

    // Calculate buffer size based on desired frequency
    const uint32_t sample_rate = SAMPLE_RATE;
    const uint32_t size = sample_rate / frequency_hz;
    const float PI_2 = 6.283185307179f;
    const float amplitude = 127.8f;

    // Create stereo buffer (interleaved L/R channels)
    std::vector<int8_t> data_buffer(size * 2);

    // Generate sine wave for left channel, cosine for right (phase difference)
    for (uint32_t i = 0; i < size; i++) {
        const float phase = PI_2 * static_cast<float>(i) / static_cast<float>(size);
        data_buffer[i*2] = static_cast<int8_t>(amplitude * sinf(phase));    // Left
        data_buffer[i*2 + 1] = static_cast<int8_t>(amplitude * cosf(phase)); // Right
    }

    /*
    // Configure PWM audio
        pwm_audio_config_t pac = {
        .gpio_num_left = LEFT_CHANNEL_GPIO,
        .gpio_num_right = RIGHT_CHANNEL_GPIO,
        .ledc_channel_left = LEDC_CHANNEL_0,
        .ledc_channel_right = LEDC_CHANNEL_1,
        .ledc_timer_sel = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .ringbuf_len = RINGBUF_SIZE
    };


    // Initialize audio system
    if (pwm_audio_init(&pac) != ESP_OK) {
        return false;
    }
    pwm_audio_set_param(sample_rate, static_cast<ledc_timer_bit_t>(8), 2);
    pwm_audio_start();
*/
    // Calculate required iterations for duration
    const uint32_t total_samples = (sample_rate * duration_ms) / 1000;
    uint32_t samples_played = 0;
    const uint32_t block_size = 256;  // Optimal write size

    // Playback loop
    while (samples_played < total_samples) {


   //        ESP_ERROR_CHECK(esp_task_wdt_reset());

        const uint32_t buffer_pos = (samples_played * 2) % data_buffer.size();
        const uint32_t remaining_in_buffer = data_buffer.size() - buffer_pos;
        const uint32_t samples_to_play = std::min({
            block_size,
            remaining_in_buffer,
            (total_samples - samples_played) * 2
        });

        size_t bytes_written = 0;
        if (pwm_audio_write(reinterpret_cast<uint8_t*>(data_buffer.data()) + buffer_pos,
                           samples_to_play,
                           &bytes_written,
                           1000 / portTICK_PERIOD_MS) != ESP_OK) {
            pwm_audio_deinit();
            return false;
        }

        samples_played += bytes_written / 2;
    }

    // Cleanup
 //   pwm_audio_stop();
 //   pwm_audio_deinit();
    return true;
}

bool initPwmAudio (){
    // Configure PWM audio
        pwm_audio_config_t pac = {
        .gpio_num_left = LEFT_CHANNEL_GPIO,
        .gpio_num_right = RIGHT_CHANNEL_GPIO,
        .ledc_channel_left = LEDC_CHANNEL_0,
        .ledc_channel_right = LEDC_CHANNEL_1,
        .ledc_timer_sel = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .ringbuf_len = RINGBUF_SIZE
    };

    // Initialize audio system
    if (pwm_audio_init(&pac) != ESP_OK) {
        return false;
    }
    pwm_audio_set_param(SAMPLE_RATE, static_cast<ledc_timer_bit_t>(8), 2);
    pwm_audio_start();

return true;
}



/**
 * @brief Task to handle audio playback. It waits for an alert level
 *        from the queue and plays the corresponding sound.
 */
void audioTask(void *parameter) {
  alert_level_t received_alert_level;
  alert_level_t last_played_level = (alert_level_t)-1;

  ESP_LOGI(TAG, "Audio Task started");

//ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

   esp_task_wdt_delete(NULL); 
  

  for (;;) {

/*
static int alert_update_period = 100;
uint32_t last_update_time = xTaskGetTickCount();
uint32_t current_time = xTaskGetTickCount();
if (current_time - last_update_time > alert_update_period ) {
last_update_time = current_time;
}
*/
//ESP_ERROR_CHECK(esp_task_wdt_reset());

   if (xQueueReceive(audioQueue, &received_alert_level, portMAX_DELAY) == pdTRUE) {
      //if (received_alert_level != last_played_level) {
        ESP_LOGI(TAG, "Audio task: Playing alert for level %d", received_alert_level);
        last_played_level = received_alert_level;


if (xSemaphoreTake(audio_mutex, portMAX_DELAY) == pdTRUE) {
 
//ESP_ERROR_CHECK(esp_task_wdt_reset());

 play_wav_from_flash(received_alert_level ) ;


 //       switch (received_alert_level) {
 //   play_sine_tone
// 
//play_sine_tone(440, 1000, 1.0f);


//play_wav_from_flash

/*
     case ALERT_VERYFAR:   play_wav_from_flash(received_alert_level); break;
          case ALERT_FAR:    play_wav_from_flash(received_alert_level); break;
          case ALERT_MEDIUMFAR: play_wav_from_flash(received_alert_level); break;
          case ALERT_MEDIUM: play_wav_from_flash(received_alert_level); break;
          case ALERT_CLOSE:   play_wav_from_flash(received_alert_level); break;
          case ALERT_IMMEDIATE: play_wav_from_flash(received_alert_level); 
                    break;
          case ALERT_SILENT:   break;
          default:            break;
        }
*/
// bool play_sine_tone(uint32_t duration_ms, uint32_t frequency, float amplitude)
/*
     case ALERT_VERYFAR:   play_sine_tone(100.0, 220, 0.9f); break;
          case ALERT_FAR:    play_sine_tone(100.0, 440, 0.7f); break;
          case ALERT_MEDIUMFAR: play_sine_tone(300.0, 466, 0.6f); break;
          case ALERT_MEDIUM: play_sine_tone(200.0, 587, 0.5f); break;
          case ALERT_CLOSE:   play_sine_tone(150.0, 659, 0.6f); break;
          case ALERT_IMMEDIATE: play_sine_tone(100, 880, 1.0f); 
          // ramping
   // for (int i = 20; i >= 0; --i) {
   //         float current_amplitude = i / 20.0f; // Ramps from 1.0 down to 0.0
   //         play_sine_tone(100, 880, current_amplitude);
    //    }

                               // vTaskDelay(pdMS_TO_TICKS(1));
                                //play_sine_tone(100, 600); 
          break;
          case ALERT_SILENT:   break;
          default:            break;
*/        

/*
     case ALERT_VERYFAR:   play_sine_tone(100.0, 220, 0.9f); break;
          case ALERT_FAR:    play_sine_tone(100.0, 440, 0.7f); break;
          case ALERT_MEDIUMFAR: play_sine_tone(300.0, 466, 0.6f); break;
          case ALERT_MEDIUM: play_sine_tone(200.0, 587, 0.5f); break;
          case ALERT_CLOSE:   play_sine_tone(150.0, 659, 0.6f); break;
          case ALERT_IMMEDIATE: play_sine_tone(100, 880, 1.0f); 
          // ramping
   // for (int i = 20; i >= 0; --i) {
   //         float current_amplitude = i / 20.0f; // Ramps from 1.0 down to 0.0
   //         play_sine_tone(100, 880, current_amplitude);
    //    }

                               // vTaskDelay(pdMS_TO_TICKS(1));
                                //play_sine_tone(100, 600); 
          break;
          case ALERT_SILENT:   break;
          default:            break;
        
*/


      
/*
          case ALERT_VERYFAR:   playAudio(100.0, 400); break;
          case ALERT_FAR:    playAudio(100.0, 400); break;
          case ALERT_MEDIUMFAR: playAudio(100.0, 700); break;
          case ALERT_MEDIUM: playAudio(200.0, 800); break;
          case ALERT_CLOSE:   playAudio(200.0, 900); break;
          case ALERT_IMMEDIATE:   playAudio(200, 1000); break;
          case ALERT_SILENT:   break;
          default:            break;
      }
          */
 //   }

   xSemaphoreGive(audio_mutex);
} else {
ESP_LOGE(TAG, "No audio_mutex available");

}


 } // got queue message

   vTaskDelay(pdMS_TO_TICKS(10));

} // loop

  
  esp_task_wdt_add(NULL);
}





void playWarningToneAsync(alert_level_t level, uint32_t duration_ms) {
    if (audioQueue == NULL) return;
    
    audio_command_t cmd = {
        .level = level,
        .duration_ms = duration_ms
    };
    
    xQueueSend(audioQueue, &cmd, pdMS_TO_TICKS(100));
   vTaskDelay(pdMS_TO_TICKS(100));

}

/*
// Updated alert handler using async playback
void update_beeper_alerts(alert_level_t new_level) {
    // Visual alerts (unchanged)
    switch (new_level) {
        case ALERT_IMMEDIATE:
            blink_task_init(RED_PIN_1, 5, 300);
            break;
        case ALERT_CLOSE:
            blink_task_init(YELLOW_PIN, 2, 500);
            break;
        default:
            break;
    }

    // Audio alerts with async playback
    switch (new_level) {
        case ALERT_VERYFAR:
            playWarningToneAsync(new_level, 300);
            break;
        case ALERT_FAR:
            playWarningToneAsync(new_level, 400);
            break;
        case ALERT_MEDIUMFAR:
            playWarningToneAsync(new_level, 500);
            break;
        case ALERT_MEDIUM:
            playWarningToneAsync(new_level, 600);
            break;
        case ALERT_CLOSE:
            playWarningToneAsync(new_level, 800);
            break;
        case ALERT_IMMEDIATE:
            playWarningToneAsync(new_level, 1000);
            break;
        default:
            playWarningToneAsync(ALERT_SILENT, 0);
            break;
    }
}
    */
///////////////////////// end tasks without caching

/*
static size_t before_free_8bit;
static size_t before_free_32bit;

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
   // TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}
*/

// #ifdef useAudio
#endif


///// vibrator

#ifdef useVibrator2
typedef struct {
    gpio_num_t pin;
    uint8_t times;
    uint32_t length_ms;
    uint32_t interval_ms;
} vibrator_task_params_t;

// Static storage for task parameters
static vibrator_task_params_t vibrator_task_params;
#endif

void resetPins (){
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 
gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 
gpio_set_level(VIBRATORPIN, 0); 
gpio_set_level(VIBRATORPIN2, 0); 
}



// **Embedded Blink Task Function**
static void vibrator_task(void *pvParameter) {
    // Configure GPIO pin as output
 //   gpio_pad_select_gpio(BLINK_PIN);
    gpio_set_direction(vibrator_task_params.pin, GPIO_MODE_OUTPUT);

//ESP_LOGI("BLINK", "blink_task %d %d %d", blink_task_params.pin,blink_task_params.times, blink_task_params.interval_ms );
//ESP_LOGI("VIBRA", "vibrator_task %d %d %d", vibrator_task_params.pin,vibrator_task_params.times, vibrator_task_params.interval_ms );
//if (xSemaphoreTake(audio_mutex, portMAX_DELAY) == pdTRUE) {
 //ESP_LOGD(TAG, "vibr audio Got mutex .");


 //  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  
    // Blink 'times' times with 'interval_ms' between blinks
    for (uint8_t i = 0; i < vibrator_task_params.times; i++) {


 //          ESP_ERROR_CHECK(esp_task_wdt_reset());


        gpio_set_level(vibrator_task_params.pin, 1);
        vTaskDelay(pdMS_TO_TICKS(vibrator_task_params.length_ms));
        gpio_set_level(vibrator_task_params.pin, 0);
        vTaskDelay(pdMS_TO_TICKS(vibrator_task_params.interval_ms ));
      //  vTaskDelay(pdMS_TO_TICKS(vibrator_task_params.interval_ms - (vibrator_task_params.interval_ms / 2)));
    }
    gpio_set_level(vibrator_task_params.pin, 0); // Final state


 //   xSemaphoreGive(audio_mutex);
//ESP_LOGD(TAG, "vibr audio gave mutex .");

    vTaskDelete(NULL);
//}

}

static void vibrator_task_init( gpio_num_t pin, uint8_t times, uint32_t length_ms, uint32_t interval_ms2) {
    vibrator_task_params.pin = pin;
    vibrator_task_params.times = times;
    vibrator_task_params.length_ms = length_ms;
    vibrator_task_params.interval_ms = interval_ms2;

//TaskHandle_t xHandle_vibrator = NULL;
// vTaskDelete( xHandle_vibrator );

 //   if (!xHandle_vibrator){

ESP_LOGI("VIBRA", "vibrator_task_init %d %d %d", pin,times, interval_ms2 );

    xTaskCreate(vibrator_task, "vibrator_task", 1024, NULL, 2, &xHandle_vibrator);
 //   }

}



#ifdef useAccelerometer
#define MPU9250_I2C_ADDRESS 0x68


// --- Filter state variables ---
// These MUST be static or global so they retain their value across task iterations.
static float current_pitch = 0.0f;
static float current_roll = 0.0f;

// A queue to safely pass the sensor data from the reader task to other tasks
QueueHandle_t mpu_data_queue;
#define MPU9250_PWR_MGMT_1_REG_ADDR   0x6B
#define MPU9250_ACCEL_XOUT_H_REG_ADDR 0x3B // Start of all sensor data

// --- MPU9250 Sensitivity Scale Factors (for default settings) ---
// Default Accel Range: ±2g -> 16384.0 LSB/g
#define ACCEL_SCALE_FACTOR 16384.0f
// Default Gyro Range: ±250 °/s -> 131.0 LSB/(°/s)
#define GYRO_SCALE_FACTOR  131.0f

// Complementary filter coefficient (ALPHA). A higher value means you trust the gyro more.
// 0.98 is a common starting point for responsive yet stable tracking.
#define COMPLEMENTARY_FILTER_ALPHA 0.98f

#endif

#ifdef useAccelerometer
/**
 * @brief FreeRTOS task to read MPU9250 sensor data and calculate a stable
 *        orientation using a complementary filter.
 * 
 * @param pvParameters The device handle for the MPU9250 I2C connection.
 */
void mpu9250_reader_task(void *pvParameters)
{

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  


    i2c_master_dev_handle_t mpu9250_dev_handle = (i2c_master_dev_handle_t)pvParameters;
    uint8_t raw_data[14]; // Buffer for all accel, temp, and gyro data
    mpu_data_t sensor_data;

  //  uint32_t last_update_time = xTaskGetTickCount();
    uint32_t last_update_time = 0;


    ESP_LOGI("MPU9250_TASK", "Fusion task started.");
   vTaskDelay(pdMS_TO_TICKS(100));
static int mpu9250_update_period = 50;
    while (1)
    {

          ESP_ERROR_CHECK(esp_task_wdt_reset());

        



        uint32_t current_time = xTaskGetTickCount();
        float dt = (float)(current_time - last_update_time) / (float)configTICK_RATE_HZ;
        
    //    last_update_time = current_time;
 //   ESP_LOGI("MPU9250_TASK", "mpu9250 check update cur %d last %d",  current_time, last_update_time );

 //  last_update_time = 0;
if (current_time - last_update_time > mpu9250_update_period ) {
   last_update_time = current_time;
  
 ESP_LOGD("MUTEX2", "mpu9250 Waiting for g_i2c_bus mutex...");
 
        ESP_LOGD("MPU9250_TASK", "mpu9250 updating");

           if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {

mpu_hasMutex = true;


  ESP_LOGD("MUTEX2", "mpu9250 i2c_mutex Mutex taken.");

  ESP_LOGD("MPU9250_TASK", "Doing mi2c_master_transmit_receive");


        esp_err_t err = i2c_master_transmit_receive(
            mpu9250_dev_handle,
            (const uint8_t[]){MPU9250_ACCEL_XOUT_H_REG_ADDR},
            1,
            raw_data,
            14,
            pdMS_TO_TICKS(100)
        );

        if (err == ESP_OK)
        {

              ESP_LOGD("MPU9250_TASK", "mi2c_master_transmit_receive success");

            // --- Parse raw data and convert to physical units ---
            int16_t raw_accel_x = (raw_data[0] << 8) | raw_data[1];
            int16_t raw_accel_y = (raw_data[2] << 8) | raw_data[3];
            int16_t raw_accel_z = (raw_data[4] << 8) | raw_data[5];
            int16_t raw_temp    = (raw_data[6] << 8) | raw_data[7]; // Temperature data
            int16_t raw_gyro_x  = (raw_data[8] << 8) | raw_data[9];
            int16_t raw_gyro_y  = (raw_data[10] << 8) | raw_data[11];
            int16_t raw_gyro_z  = (raw_data[12] << 8) | raw_data[13];

            // Convert raw values
            sensor_data.accel_x_g = (float)raw_accel_x / ACCEL_SCALE_FACTOR;
            sensor_data.accel_y_g = (float)raw_accel_y / ACCEL_SCALE_FACTOR;
            sensor_data.accel_z_g = (float)raw_accel_z / ACCEL_SCALE_FACTOR;
            sensor_data.gyro_x_dps  = (float)raw_gyro_x / GYRO_SCALE_FACTOR;
            sensor_data.gyro_y_dps  = (float)raw_gyro_y / GYRO_SCALE_FACTOR;
            sensor_data.gyro_z_dps  = (float)raw_gyro_z / GYRO_SCALE_FACTOR;
            
            // ***************************************************************
            // *** ADDED THIS LINE BACK IN ***
            // Temperature formula from the MPU9250 register map datasheet
            sensor_data.temp_c  = ((float)raw_temp / 333.87f) + 21.0f;
            // ***************************************************************

            // --- Calculate Pitch and Roll from Accelerometer ---
            float pitch_from_accel = atan2f(sensor_data.accel_y_g, sensor_data.accel_z_g) * (180.0f / M_PI);
            float roll_from_accel = atan2f(-sensor_data.accel_x_g, sqrtf(sensor_data.accel_y_g * sensor_data.accel_y_g + sensor_data.accel_z_g * sensor_data.accel_z_g)) * (180.0f / M_PI);

            // --- THE COMPLEMENTARY FILTER ---
            current_pitch = COMPLEMENTARY_FILTER_ALPHA * (current_pitch + sensor_data.gyro_x_dps * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * pitch_from_accel;
            current_roll  = COMPLEMENTARY_FILTER_ALPHA * (current_roll + sensor_data.gyro_y_dps * dt) + (1.0f - COMPLEMENTARY_FILTER_ALPHA) * roll_from_accel;

            // --- Populate the struct and send it to the queue ---
            sensor_data.pitch = current_pitch;
            sensor_data.roll = current_roll;
            
            xQueueSend(mpu_data_queue, &sensor_data, 0);

        } else {
            xSemaphoreGive(i2c_mutex);
            ESP_LOGE("MPU9250_TASK", "Failed to read sensor data: %s", esp_err_to_name(err));
        }

//////  fall detection
    float total_accel_magnitude = sqrtf(
        sensor_data.accel_x_g * sensor_data.accel_x_g +
        sensor_data.accel_y_g * sensor_data.accel_y_g +
        sensor_data.accel_z_g * sensor_data.accel_z_g
    );
 ESP_LOGD("MPU9250_TASK", "total_accel_magnitude: %f", total_accel_magnitude);



// When the device is still, magnitude should be ~1.0. During free-fall, it's ~0.0.
    if (total_accel_magnitude >= fall_magnitude) { // Threshold for detecting free-fall
        ESP_LOGI("FALL_DETECT", "FREE-FALL DETECTED! Magnitude: %.2f g", total_accel_magnitude);
 //   blink_task_init(GPIO_NUM_35, 2, 500); // 
 blink_task_init(GPIO_NUM_36, 5, 500); // 
//  blink_task_init(GPIO_NUM_37, 2, 500); // 

 // vTaskDelay(pdMS_TO_TICKS(1000));

    // You could trigger an alert here or set a flag for another task to see.
    }

               
//if (mpu_hasMutex){
            xSemaphoreGive(i2c_mutex);
            mpu_hasMutex = false;
              ESP_LOGD("MUTEX2", "mpu9250 g_i2c_bus Mutex given.");
//}
      
    }

} // mpu9250_update_period


//xSemaphoreGive(g_i2c_bus_mutex);
   vTaskDelay(pdMS_TO_TICKS(100));
}
    
     
}
#endif

#ifdef useAccelerometer
/**
 * @brief Example task that waits for and processes MPU9250 data from the queue.
 *        This task demonstrates how to correctly access the fused orientation data.
 */

void data_processor_task(void *pvParameters)
{

      //  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  // esp_task_wdt_delete(NULL); 
  


    mpu_data_t received_data;
    ESP_LOGI("PROCESSOR_TASK", "Task started, waiting for data.");

    while (1)
    {

 //         ESP_ERROR_CHECK(esp_task_wdt_reset());

        // Wait indefinitely until an item is available on the queue.
        if (xQueueReceive(mpu_data_queue, &received_data, portMAX_DELAY) == pdPASS)
        {


       
// ESP_ERROR_CHECK(esp_task_wdt_reset());

 ESP_LOGD("PROCESSOR_TASK", "received data.");
/*
         straight    Pitch: -25.00°, Roll: 159.13° 
          up   Pitch: -48.69°, Roll: 124.07°
        down Pitch: -53.90°, Roll: 179.25°
        lean right: Pitch: -14.88°, Roll: 186.32°
        lean left: Pitch: -40.04°, Roll: 130.50°
turn right: Pitch: -38.04°, Roll: 161.72°
turn left: Pitch: -68.80°, Roll: 162.69°
*/


/*
if (received_data.pitch < -50 ){
    ESP_LOGI("ACCEL", "LEFT");
}

if (received_data.pitch < -40 ){
    ESP_LOGI("ACCEL", "UP");
}


if (received_data.roll > 130 ){
    ESP_LOGI("ACCEL", "lean right");
}
if (received_data.roll > 170 ){
    ESP_LOGI("ACCEL", "lean right");
}
*/

uint32_t current_time = xTaskGetTickCount();

if ( abs(received_data.pitch -  last_pitch ) > acc_Move_limit || abs(received_data.roll -  last_roll) > acc_Move_limit ) {

 ESP_LOGI("PROCESSOR_TASK", "Device is moved %.1f  %.1f current_time: %d last_movement: %d", abs(received_data.pitch -  last_pitch ), abs(received_data.roll -  last_roll), current_time, last_movement   );

#ifdef useAudio
//update_beeper_alerts(ALERT_MEDIUMFAR, 1);
//playAudio(500, 1200);
//playAudio(500, 1400);
#endif

 // PROCESSOR_TASK: Device is moved (Roll: -21.1). (Pitch: 5.0)  26.4  8.1 time: 7089
// PROCESSOR_TASK: Device is moved (Roll: -26.9). (Pitch: 15.3)  25.3  13.9 current_time: 1374 last_movement: 1333
// PROCESSOR_TASK: Device is moved (Roll: -14.0). (Pitch: 93.5)  1.0  32.7 current_time: 9440 last_movement: 1374
 //   uint32_t current_time = xTaskGetTickCount();
   // 35.4  7.0  current_time: 27785 last_movement: 10777


   //     float dt = (float)(current_time - last_update_time) / (float)configTICK_RATE_HZ;
    
 //if (( current_time - last_movement ) > movement_timeout_sec * 1000){


// if (!vl53l5cx_running ){
// start vl53l5cx task
        // 3. Give the semaphore. This will unblock the worker_task.
 //       ESP_LOGW("PROCESSOR_TASK", "Signaling the worker task to start!");
  //      xSemaphoreGive(g_binary_semaphore);
#ifdef useVl53l5cx
    ESP_LOGD("PROCESSOR_TASK", "Sending START signal to xHandle_vl53l5cx.");
     xTaskNotify(xHandle_vl53l5cx, START_BIT, eSetBits);
#endif

#ifdef useUltrasound
 ESP_LOGI("PROCESSOR_TASK", "Sending START signal to xHandle_ultrasonic.");
  xTaskNotify( xHandle_ultrasonic, START_BIT, eSetBits);
#endif
  #ifdef useUltrasound_2
   ESP_LOGI("PROCESSOR_TASK", "Sending START signal to xHandle_ultrasonic 2.");
  xTaskNotify( xHandle_ultrasonic_2, START_BIT, eSetBits);
#endif

#ifdef useGPS
ESP_LOGD("PROCESSOR_TASK", "Sending START signal to xHandle_GPS.");
xTaskNotify( xHandle_GPS, START_BIT, eSetBits);
#endif

// 1. Declare a handle for the worker task
//TaskHandle_t g_worker_task_handle = NULL;

// xTaskNotifyGive(g_worker_task_handle);
// ESP_LOGI("PROCESSOR_TASK", "vTaskResume( xHandle_vl53l5cx )");
//vTaskResume( xHandle_vl53l5cx );
//vl53l5cx_running = true;
//}

 last_movement = current_time;

} // > acc_Move_limit



if (( current_time - last_movement ) > movement_timeout_msec * 1000){ //vl53l5cx_running && 
// stop vl53l5cx task
// ESP_LOGI("PROCESSOR_TASK", "vTaskSuspend( xHandle_vl53l5cx )");
 ESP_LOGD("PROCESSOR_TASK", "no movement, current_time %d last_movement %d", current_time, last_movement);

resetPins();

 //xSemaphoreGive(g_i2c_bus_mutex);
//ESP_LOGD("MUTEX1", "Given g_i2c_bus mutex before vTaskSuspend");
#ifdef useVl53l5cx
 ESP_LOGD("PROCESSOR_TASK", "Sending STOP signal to worker.");
 xTaskNotify(xHandle_vl53l5cx, STOP_BIT, eSetBits);
 #endif

 #ifdef useUltrasound
 ESP_LOGI("PROCESSOR_TASK", "Sending STOP_BIT  to xHandle_ultrasonic .");
    xTaskNotify( xHandle_ultrasonic, STOP_BIT, eSetBits);
#endif

#ifdef useUltrasound_2
    xTaskNotify( xHandle_ultrasonic_2, STOP_BIT, eSetBits);
#endif

#ifdef useGPS
ESP_LOGD("PROCESSOR_TASK", "Sending STOP signal to xHandle_GPS.");
xTaskNotify( xHandle_GPS, STOP_BIT, eSetBits);
#endif

//vTaskSuspend( xHandle_vl53l5cx );
//vl53l5cx_running = false;
     
} // 


// ESP_LOGI("VL53_TASK", "Device is moved (Roll: %.1f). (Pitch: %.1f)  %.1f  %.1f", current_orientation.pitch ,current_orientation.roll, abs(current_orientation.pitch -  last_pitch ), abs(current_orientation.roll -  last_roll)  );

last_pitch = received_data.pitch;
last_roll = received_data.roll;
/*
            if (current_orientation.pitch > 50.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed RIGHT (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.pitch < -40.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed LEFT (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }


            if (current_orientation.roll > 140) {
                ESP_LOGI("VL53_TASK", "Device is pointed DOWN (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.roll < 100) {
                ESP_LOGI("VL53_TASK", "Device is pointed UP (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }
*/





            // --- FUSED ORIENTATION DATA (Most Important Output) ---
           ESP_LOGD("PROCESSOR_TASK", "Orientation -> Pitch: %.2f°, Roll: %.2f°", received_data.pitch, received_data.roll);
        }
 vTaskDelay(pdMS_TO_TICKS(20));


    } // while 1
  //     esp_task_wdt_delete(NULL); 
  
//  esp_task_wdt_add(NULL);
}

#endif



#ifdef useRGBLed
void rgb_led_init() {
    ESP_LOGI("LED_INIT", "Initializing WS2812 LED strip...");
    
    // RMT (Remote Control) peripheral configuration for the LED strip
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz resolution, good for WS2812
        .flags = {
            .with_dma = false, // DMA is not needed for a single LED
        }
    };

    // LED strip configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,           // The GPIO the LED is connected to
        .max_leds = 1,                           // The number of LEDs in the strip
        .led_model = LED_MODEL_WS2812,           // Set the LED model to WS2812
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Set the color order
    };

    // Create the LED strip handle
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
 
    if (err != ESP_OK || led_strip == NULL) {
        ESP_LOGE("LED_INIT", "LED strip init failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("LED_INIT", "LED strip initialized successfully.");
        // Clear the strip to ensure it's off at boot.
        led_strip_clear(led_strip);
    }
}



static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = WS2812_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
               .flags = {
            .with_dma = false, // DMA is not needed for a single LED
        }

//        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
 led_strip_clear(led_strip);
}

const char* led_color_to_string(LedColor color) {
    switch (color) {
        case LedColor::BLACK:    return "BLACK";
        case LedColor::GREEN:    return "GREEN";
        case LedColor::YELLOW:   return "YELLOW";
        case LedColor::ORANGE:  return "ORANGE";
        case LedColor::RED:     return "RED";
        case LedColor::WHITE:   return "WHITE"; // Corrected position
        case LedColor::BLUE:    return "BLUE";
        case LedColor::CYAN:    return "CYAN"; // Added
        case LedColor::MAGENTA: return "MAGENTA"; // Added
        default:                return "UNKNOWN_COLOR";
    }
}

void rgb_led_off() {
    // Take the mutex. Wait up to 10ms to get it.
    if (xSemaphoreTake(led_strip_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        led_strip_clear(led_strip);

        // ALWAYS give the mutex back.
        xSemaphoreGive(led_strip_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to get LED mutex in off!");
    }
}
 
 void rgb_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
     esp_err_t  err ;
  ESP_LOGI("LED", "rgb_led_set_color %d %d %d", r, g, b);
     if (led_strip == nullptr) {
         ESP_LOGE("LED", "led_strip is NULL, skipping rgb_led_set_color");
         return;
     }

  led_strip_clear(led_strip);
  

       err =  led_strip_set_pixel(led_strip, 0, r, g, b);
       if (err != ESP_OK || led_strip == NULL) {
         ESP_LOGE("LED", "led_strip_set_pixel failed: %s", esp_err_to_name(err));
     } else {
        // ESP_LOGI("LED", "led_strip_set_pixel successful: %s", esp_err_to_name(err));
  //   ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Turn off initially
     }
     
  
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

  if (xSemaphoreTake(led_strip_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

     if (brightness < 0.0f) brightness = 0.0f;
     if (brightness > 1.0f) brightness = 1.0f;
 
     uint8_t scaled_r = gamma_correct((uint8_t)(r * brightness));
     uint8_t scaled_g = gamma_correct((uint8_t)(g * brightness));
     uint8_t scaled_b = gamma_correct((uint8_t)(b * brightness));
 
        ESP_LOGI(TAG, "rgb_led_set_color_with_brightness: %d %d %d %f", scaled_r, scaled_g, scaled_b, brightness);

     //  led_strip_clear(led_strip);
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, scaled_r, scaled_g, scaled_b));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

         // ALWAYS give the mutex back.
        xSemaphoreGive(led_strip_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to get LED mutex in set_color!");
    }

 }
 
 void rgb_led_blink(LedColor color, int times, int ms_delay, float strength_val) {
     uint8_t r = 0, g = 0, b = 0;
 
     switch (color) {
         case LedColor::RED:      r = 255; g = 0;   b = 0;   break;
         case LedColor::ORANGE:   r = 255; g = 140;   b = 0;   break; //255,140,0
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
        ESP_LOGI("rgb_led_blink", "repeat %d of %d", i, times );
         rgb_led_set_color_with_brightness(r, g, b, strength_val);
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
      //   rgb_led_off();
         vTaskDelay(pdMS_TO_TICKS(ms_delay));
     }
 }
 
void rgb_led_constant(LedColor color, float strength_val) {
     uint8_t r = 0, g = 0, b = 0;
 
     switch (color) {
         case LedColor::RED:      r = 255; g = 0;   b = 0;   break;
         case LedColor::ORANGE:   r = 255; g = 140;   b = 0;   break; //255,140,0
         case LedColor::GREEN:    r = 0;   g = 255; b = 0;   break; 
         case LedColor::BLUE:     r = 0;   g = 0;   b = 255; break;
         case LedColor::YELLOW:   r = 255; g = 255; b = 0;   break;
         case LedColor::CYAN:     r = 0;   g = 255; b = 255; break;
         case LedColor::MAGENTA:  r = 180; g = 100;   b = 255; break;
         case LedColor::WHITE:    r = 255; g = 255; b = 255; break; 
         case LedColor::BLACK:    r = 0;   g = 0;   b = 0;   break;
         default:                 r = 0;   g = 0;   b = 0;   break;
     }
 
         ESP_LOGI(TAG, "rgb_led_constant: Color:%d %d %d", r, g, b);

  //   for (int i = 0; i < times; ++i) {
       vTaskDelay(pdMS_TO_TICKS(10));
         rgb_led_set_color_with_brightness(r, g, b, strength_val);
       vTaskDelay(pdMS_TO_TICKS(5));
   //      rgb_led_off();
   //      vTaskDelay(pdMS_TO_TICKS(ms_delay));
 //    }
 }
 
#endif

#ifdef useUart 
void init_uart(void) {
    // Configure UART parameters
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Apply UART configuration
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver with RX buffer and a small TX buffer
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUFFER_SIZE, 512, 0, NULL, 0));

    // Set UART mode explicitly
    ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_UART));
}
#endif

static beeper_state_t beeper = {};

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
alert_level_t get_alert_level(uint16_t distance_cm, uint16_t measured_average_cm) {

    // change levels if theres nothing near
uint16_t average_cm_far = 250;
uint16_t average_cm_close = 100;
uint16_t average_cm_veryclose = 50;
/*
 if (measured_average_cm > average_cm_far){
ALERT_IMMEDIATE_LIMIT = 100;
ALERT_CLOSE_LIMIT  = 140;
ALERT_MEDIUM_LIMIT  = 180;
ALERT_MEDIUMFAR_LIMIT  = 220;
ALERT_FAR_LIMIT  = 250;
ALERT_VERYFAR_LIMIT  = 300;
    }

 if (measured_average_cm < average_cm_close){
ALERT_IMMEDIATE_LIMIT = 20;
ALERT_CLOSE_LIMIT  = 40;
ALERT_MEDIUM_LIMIT  = 60;
ALERT_MEDIUMFAR_LIMIT  = 80;
ALERT_FAR_LIMIT  = 100;
ALERT_VERYFAR_LIMIT  = 120;
    }
 if (measured_average_cm < average_cm_veryclose){
ALERT_IMMEDIATE_LIMIT = 10;
ALERT_CLOSE_LIMIT  = 20;
ALERT_MEDIUM_LIMIT  = 30;
ALERT_MEDIUMFAR_LIMIT  = 40;
ALERT_FAR_LIMIT  = 50;
ALERT_VERYFAR_LIMIT  = 60;
    }
*/

/*
ALERT_IMMEDIATE_LIMIT = 30; // measured_average_cm / 8
ALERT_CLOSE_LIMIT  = measured_average_cm / 6;
ALERT_MEDIUM_LIMIT  = measured_average_cm;
ALERT_MEDIUMFAR_LIMIT  = measured_average_cm * 1.25;
ALERT_FAR_LIMIT  = measured_average_cm * 2;
ALERT_VERYFAR_LIMIT  = measured_average_cm * 3;
*/


if (indoor){
    // configure alert distances
int ALERT_IMMEDIATE_LIMIT = 30;
int ALERT_VERYCLOSE_LIMIT =40;
int ALERT_CLOSE_LIMIT  = 50;
int ALERT_MEDIUM_LIMIT  = 80;
int ALERT_MEDIUMFAR_LIMIT  = 100;
int ALERT_FAR_LIMIT  = 150;
int ALERT_VERYFAR_LIMIT  = 200;
}


    if (distance_cm <= ALERT_IMMEDIATE_LIMIT ) { //+ HYSTERESIS
        return ALERT_IMMEDIATE;
        } else 
            if (distance_cm <= ALERT_VERYCLOSE_LIMIT ) { //+ HYSTERESIS
        return ALERT_VERYCLOSE;
    } else if (distance_cm <= ALERT_CLOSE_LIMIT ) {
        return ALERT_CLOSE;
    } else if (distance_cm <= ALERT_MEDIUM_LIMIT ) {
        return ALERT_MEDIUM;
      } else if(distance_cm <= ALERT_MEDIUMFAR_LIMIT){
        return(ALERT_MEDIUMFAR);
    } else if (distance_cm <= ALERT_FAR_LIMIT ) {
        return ALERT_FAR;
   } else if (distance_cm <= ALERT_VERYFAR_LIMIT) {
        return ALERT_VERYFAR;

    } else {
        return ALERT_SILENT;
    }
}

#ifdef useUart 
void uart_task(void *pvParameters) {
    uint8_t data[4];

 ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE) {


    while (1) {

        ESP_ERROR_CHECK(esp_task_wdt_reset());

        // Read 4 bytes from UART
        int len = uart_read_bytes(UART_NUM, data, 4, pdMS_TO_TICKS(UART_TIMEOUT_MS));

        if (len == 4 && data[0] == 0xFF) {
            // Calculate checksum
            int sum = (data[0] + data[1] + data[2]) & 0xFF;
            if (sum == data[3]) {
                // Convert distance to centimeters
                int distance = (data[1] << 8) | data[2];

                if (distance > 30) {
                    ESP_LOGI(TAG, "Distance: %.2f cm", distance / 10.0);
                } else {
                    ESP_LOGW(TAG, "Below the lower limit");
                }
            } else {
                ESP_LOGE(TAG, "Checksum error (Received: 0x%02X, Expected: 0x%02X)", data[3], sum);
            }
        } else if (len > 0) {
            ESP_LOGW(TAG, "Invalid data received");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

  xSemaphoreGive(uart_mutex);

} // mutex
}
#endif


static uint32_t last_update_time = xTaskGetTickCount();
static int alert_update_period = 50;
//


// Updated alert handler using async playback
void update_beeper_alerts(alert_level_t new_level, int direction) {
    // Visual alerts (unchanged)

//uint32_t last_update_time = xTaskGetTickCount();
alwaysBeep = true;
resetPins();

   // Check if alert level changed
    if (new_level != beeper.current_level || alwaysBeep) {
        beeper.current_level = new_level;
        beeper.last_beep_time = 0;
        beeper.is_beeping = false;
        set_beeper_tone(0, false);
     

    if (beeper.current_level == ALERT_SILENT) {
        if (beeper.is_beeping) {
            set_beeper_tone(0, false);
            beeper.is_beeping = false;
        }




      //  return;
    } 
    

uint32_t current_time = xTaskGetTickCount();

if(new_level < 4){
alert_update_period = 80;
} 
if(new_level == 4){
alert_update_period = 60;
} 
if(new_level == 5){
alert_update_period = 50;
} 


if(new_level == 6){
alert_update_period = 40;
} 

if(new_level > 6){
alert_update_period = 20;
} else {
//  alert_update_period = 60;  
}
//alert_update_period = 10;


//ESP_LOGI(TAG, "update period:%d current_time:%d last_update_time:%d", alert_update_period, current_time, last_update_time);
// period:70 current_time:4643 last_update_time:4543

if (current_time - last_update_time > alert_update_period ) {
last_update_time = current_time;

// ESP_LOGI(TAG, "update period current_time:%d last_update_time:%d", current_time, last_update_time);

 ESP_LOGI(TAG, "new_level %d dir:%d", new_level, direction);

 // if (audioQueue != NULL) {
   //   xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 // }


if (new_level == ALERT_VERYFAR ){
   // xQueueSend(audioQueue, &new_level, portMAX_DELAY);
//playWarningToneAsync(new_level, 200);
return;
}



if (new_level != ALERT_VERYFAR ){
   // xQueueSend(audioQueue, &new_level, portMAX_DELAY);
//playWarningToneAsync(new_level, 200);

}

/*
    if (audio_state.queue) {
         ESP_LOGI(TAG, "xQueueSendFromISR %d ", new_level);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(audio_state.queue, &new_level, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
*/

//static bool playAudio = true;
//static bool useVibratorAlerts = false;



if (direction == 1){

//resetPins();


    switch (new_level) {
        case ALERT_IMMEDIATE:
           // blink_task_init(RED_PIN_1, 1, 300);
             gpio_set_level(RED_PIN_1, 1); 
             gpio_set_level(BLUE_PIN_1, 0); 
             gpio_set_level(GREEN_PIN_1, 0); 
           // blink_task_init(RED_PIN, 2, 300); 
             #ifdef useVibrator2
             if(useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN, 1, 80, 50);
             }
        #endif
 //       xQueueOverwrite(audioAlertQueue, &new_level);
 #ifdef useAudio
 if(useAudioAlerts){
 ESP_LOGI(TAG, "sens1 xQueueSend to audioQueue", new_level);
 
xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif
          //   ESP_LOGI(TAG, "new_level %d dir:%d", new_level, direction);

            break;

            case ALERT_VERYCLOSE:
                #ifdef useVibrator2
        if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN, 2, 60, 20);
        }
        #endif

  #ifdef useAudio
 if(useAudioAlerts){
 xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif   
             gpio_set_level(RED_PIN_1, 1); 
             gpio_set_level(BLUE_PIN_1, 1); 
             gpio_set_level(GREEN_PIN_1, 0); 
break;

        case ALERT_CLOSE:
        #ifdef useVibrator2
        if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN, 2, 60, 20);
        }
        #endif

         #ifdef useAudio
 if(useAudioAlerts){
 xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif

         //   playWarningToneAsync(new_level, 800);
            //blink_task_init(RED_PIN_1, 1, 300); //
           gpio_set_level(RED_PIN_1, 0); 
           gpio_set_level(BLUE_PIN_1, 1); 
           gpio_set_level(GREEN_PIN_1, 0); 
            // xQueueSend(audioQueue, &new_level, portMAX_DELAY);
            break;

         case ALERT_MEDIUM:
         #ifdef useVibrator2
         if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN, 1, 50, 50);
         }
        #endif
            gpio_set_level(RED_PIN_1, 0); 
            gpio_set_level(BLUE_PIN_1, 1);  
            gpio_set_level(GREEN_PIN_1, 1); 

#ifdef useAudio           

 if(useAudioAlerts){
 xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif
            break;

         case ALERT_MEDIUMFAR:
         #ifdef useVibrator2
         if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN, 1, 50, 100);
         }
        #endif
            gpio_set_level(RED_PIN_1, 0); 
            gpio_set_level(BLUE_PIN_1, 0);  
            gpio_set_level(GREEN_PIN_1, 1); 
            
 //xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 #ifdef useAudio
if(useAudioAlerts){
//xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif
            break;
            
       case ALERT_FAR:
       #ifdef useVibrator2
      //  vibrator_task_init( VIBRATORPIN, 1, 40, 200);
        #endif
// vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(RED_PIN_1, 0); 
        gpio_set_level(BLUE_PIN_1, 0);  
        gpio_set_level(GREEN_PIN_1, 0); 
        
 #ifdef useAudio
if(useAudioAlerts){
//xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif
            break;

       case ALERT_VERYFAR:
       #ifdef useVibrator2
      //  vibrator_task_init( VIBRATORPIN, 1, 40, 200);
        #endif
// vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(RED_PIN_1, 0); 
        gpio_set_level(BLUE_PIN_1, 0);  
        gpio_set_level(GREEN_PIN_1, 0); 
          //  playWarningToneAsync(new_level, 400);
            break;

        default:
       // playWarningToneAsync(ALERT_SILENT, 0);
        gpio_set_level(RED_PIN_1, 0); 
        gpio_set_level(BLUE_PIN_1, 0);  
        gpio_set_level(GREEN_PIN_1, 0); 
            break;
    }

/*
    // Audio alerts with async playback
    switch (new_level) {
        case ALERT_VERYFAR:
            playWarningToneAsync(new_level, 300);
            break;
        case ALERT_FAR:
            playWarningToneAsync(new_level, 400);
            break;
        case ALERT_MEDIUMFAR:
            playWarningToneAsync(new_level, 500);
            break;
        case ALERT_MEDIUM:
            playWarningToneAsync(new_level, 600);
            break;
        case ALERT_CLOSE:
            playWarningToneAsync(new_level, 800);
            break;
        case ALERT_IMMEDIATE:
        blink_task_init(RED_PIN_1, 5, 300); // 
#ifdef useVibrator2
vibrator_task_init( VIBRATORPIN, 1, 200, 100);
#endif

#ifdef useAudio
//playWarningToneAsync(new_level, 1000);
#endif

           // playWarningToneAsync(new_level, 1000);
            break;
        default:
            playWarningToneAsync(ALERT_SILENT, 0);
            break;
    }
            */
} // direction 1


else { //////////// direction 2




    switch (new_level) {
        case ALERT_IMMEDIATE:
           // blink_task_init(RED_PIN, 2, 300);
            gpio_set_level(RED_PIN, 1); 
            gpio_set_level(BLUE_PIN, 0);  
            gpio_set_level(GREEN_PIN, 0); 
             #ifdef useVibrator2
             if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN2, 4, 60, 20);
             }
        #endif
 //       xQueueOverwrite(audioAlertQueue, &new_level);
 #ifdef useAudio
if(useAudioAlerts){
xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif
          //   ESP_LOGI(TAG, "new_level %d dir:%d", new_level, direction);

            break;

                    case ALERT_VERYCLOSE:
           // blink_task_init(RED_PIN, 2, 300);
            gpio_set_level(RED_PIN, 1); 
            gpio_set_level(BLUE_PIN, 0);  
            gpio_set_level(GREEN_PIN, 0); 
             #ifdef useVibrator2
             if (useVibratorAlerts){
               // vibrator_task_init( VIBRATORPIN2, 3, 80, 50);
            vibrator_task_init( VIBRATORPIN2, 2, 80, 40);
             }
        #endif
 //       xQueueOverwrite(audioAlertQueue, &new_level);
 #ifdef useAudio
if(useAudioAlerts){
xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
#endif
          //   ESP_LOGI(TAG, "new_level %d dir:%d", new_level, direction);

            break;

        case ALERT_CLOSE:
        #ifdef useVibrator2
        if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN2, 1, 100, 60);
        }
        #endif

         #ifdef useAudio
            if(useAudioAlerts){
xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }
        #endif

         //   playWarningToneAsync(new_level, 800);
           // blink_task_init(RED_PIN, 2, 300); //
            gpio_set_level(RED_PIN, 1); 
            gpio_set_level(BLUE_PIN, 1);  
            gpio_set_level(GREEN_PIN, 0); 
            // xQueueSend(audioQueue, &new_level, portMAX_DELAY);
            break;

         case ALERT_MEDIUM:
         #ifdef useVibrator2
         if (useVibratorAlerts){
            vibrator_task_init( VIBRATORPIN2, 1, 50, 70);
         }
        #endif
            gpio_set_level(RED_PIN, 0); 
            gpio_set_level(BLUE_PIN, 1);  
            gpio_set_level(GREEN_PIN, 0); 

#ifdef useAudio           

// ESP_LOGI(TAG, "sens1 xQueueSend to audioQueue", new_level);
if(useAudioAlerts){
xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 }

#endif
            break;

         case ALERT_MEDIUMFAR:
         #ifdef useVibrator2
         if (useVibratorAlerts){
          //  vibrator_task_init( VIBRATORPIN2, 1, 50, 100);
         }
        #endif
            gpio_set_level(RED_PIN, 0); 
            gpio_set_level(BLUE_PIN, 0);  
            gpio_set_level(GREEN_PIN, 1); 
            
 //xQueueSend(audioQueue, &new_level, portMAX_DELAY);
 #ifdef useAudio
 //ESP_LOGI(TAG, "sens1 xQueueSend to audioQueue", new_level);

//xQueueSend(audioQueue, &new_level, portMAX_DELAY);
#endif
            break;
            
       case ALERT_FAR:
       #ifdef useVibrator2
      //  vibrator_task_init( VIBRATORPIN, 1, 40, 200);
        #endif
// vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(RED_PIN, 0); 
        gpio_set_level(BLUE_PIN, 0);  
        gpio_set_level(GREEN_PIN, 0); 
        
 #ifdef useAudio
 //ESP_LOGI(TAG, "sens1 xQueueSend to audioQueue", new_level);

//xQueueSend(audioQueue, &new_level, portMAX_DELAY);
#endif
            break;

       case ALERT_VERYFAR:
       #ifdef useVibrator2
      //  vibrator_task_init( VIBRATORPIN, 1, 40, 200);
        #endif
// vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(RED_PIN, 0); 
        gpio_set_level(BLUE_PIN, 0);  
        gpio_set_level(GREEN_PIN, 0); 
          //  playWarningToneAsync(new_level, 400);
            break;

        default:
       // playWarningToneAsync(ALERT_SILENT, 0);
        gpio_set_level(RED_PIN, 0); 
        gpio_set_level(BLUE_PIN, 0);  
        gpio_set_level(GREEN_PIN, 0); 
            break;
    }


//vTaskDelay(pdMS_TO_TICKS(200));
} // end sensor2

} // update time 

} // new level check

}

/*
void system_monitor_task(void *pvParameters) {
    while(1) {
        printf("### Task List ###\n");
        vTaskList();
        printf("### Runtime Stats ###\n");
        vTaskGetRunTimeStats( NULL, 100, NULL, NULL );
        printf("####################\n\n");
        vTaskDelay( pdMS_TO_TICKS( 60000 )); // Log every minute
    }
}
*/

#ifdef useVl53l5cx

bool vl53l5cx_recover(VL53L5CX_Configuration *dev) {
    ESP_LOGW(TAG, "Attempting VL53L5CX recovery...");
    
    // Take I2C mutex for recovery operations
     ESP_LOGD("MUTEX3", "Waiting for I2C Mutex for recovery.");
    if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE("MUTEX3", "Could not get I2C mutex for recovery");
        vl53l5cx_hasMutex = false;
        return false;
    }
    ESP_LOGD("MUTEX3", "I2C Mutex taken for recovery.");
vl53l5cx_hasMutex = true;

    uint8_t status;
    uint8_t isAlive;
    
    // Step 1: Stop any ongoing ranging
    ESP_LOGI(TAG, "Stopping ranging...");
    vl53l5cx_stop_ranging(dev);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Step 2: Hardware reset
    ESP_LOGI(TAG, "Performing hardware reset...");
    VL53L5CX_Reset_Sensor(&(dev->platform));
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for reset to complete
    
    // Step 3: Check if sensor is alive after reset
    ESP_LOGI(TAG, "Checking sensor status after reset...");
    status = vl53l5cx_is_alive(dev, &isAlive);
    if (!isAlive || status) {
        ESP_LOGE(TAG, "Sensor not alive after reset. Status: %d, Alive: %d", status, isAlive);

        if (vl53l5cx_hasMutex){
        xSemaphoreGive(g_i2c_bus_mutex);
        vl53l5cx_hasMutex = false;
         ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given 3.");
        }
        return false;
    }
    
    // Step 4: Re-initialize the sensor
    ESP_LOGI(TAG, "Re-initializing sensor...");
    status = vl53l5cx_init(dev);
    if (status) {
        ESP_LOGE(TAG, "Re-initialization failed with status: %d", status);
         if (vl53l5cx_hasMutex){
        xSemaphoreGive(g_i2c_bus_mutex);
        vl53l5cx_hasMutex = false;
         ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given 4.");
         }
        return false;
    }
    
    // Step 5: Reconfigure the sensor
    ESP_LOGI(TAG, "Reconfiguring sensor...");
    status = vl53l5cx_set_resolution(dev, VL53L5CX_RESOLUTION_4X4);
    if (status) {
        ESP_LOGW(TAG, "Failed to set resolution during recovery: %d", status);
    }
    
    status = vl53l5cx_set_ranging_frequency_hz(dev, 15);
    if (status) {
        ESP_LOGW(TAG, "Failed to set frequency during recovery: %d", status);
    }
    
    // Step 6: Restart ranging
    ESP_LOGI(TAG, "Restarting ranging...");
    status = vl53l5cx_start_ranging(dev);
    if (status) {
        ESP_LOGE(TAG, "Failed to restart ranging: %d", status);
        xSemaphoreGive(g_i2c_bus_mutex);
         ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given 6.");
        return false;
    }
    
    // Release mutex
     if (vl53l5cx_hasMutex){
    xSemaphoreGive(g_i2c_bus_mutex);
    vl53l5cx_hasMutex = false;
    ESP_LOGD("MUTEX3", "g_i2c_bus_mutex Mutex given from recover.");
     }
    ESP_LOGI(TAG, "VL53L5CX recovery successful!");
    return true;
}


// New task function for the sensor loop
void vl53l5cx_reader_task(void *pvParameters) {

ESP_LOGI(TAG, "vl53l5cx_reader_task started. Will wait for a signal to run.");


    uint32_t minStackBytesRemaining = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Sensor Task: Initial Stack Remaining %d bytes", minStackBytesRemaining);
    static uint32_t runCounter = 0;
   
   // --- Add a counter for consecutive "not ready" failures ---
    int not_ready_count = 0;
    const int NOT_READY_THRESHOLD = 10; // Trigger recovery after 10 failures (~500ms)

    // gyro stuff
    mpu_data_t current_orientation;
    // Set a default orientation in case we haven't received one yet
    current_orientation.pitch = 0;
    current_orientation.roll = 0;



    sensor_task_params_t *params = (sensor_task_params_t *)pvParameters;
    VL53L5CX_Configuration *Dev = params->dev;
    const uint8_t (*gridLayout)[4] = params->gridLayout;
    alert_level_t *gridAlerts = params->gridAlerts;
    
    uint8_t status, isReady, i;
    VL53L5CX_ResultsData Results;
    uint32_t last_sensor_read = 0;
    const uint32_t SENSOR_READ_INTERVAL = 600; // 1ms
    int loop = 0;
    uint16_t primary_distance_total = 0;

   // Static arrays for grid layout and alerts
    static const uint8_t gridTop[4] = {
        0, 4, 8, 12
    };
    int gridTop_arr_size = sizeof(gridTop)/sizeof(gridTop[0]);


static const uint8_t gridBottom[4] = {
        3, 7, 11, 15
    };

int gridBottom_arr_size = sizeof(gridBottom)/sizeof(gridBottom[0]);

  // Static arrays for grid layout and alerts
    static const uint8_t gridLeft[2] = {
        1, 2
    };
static const uint8_t gridRight[2] = {
         13, 14
    };

static const uint8_t gridCentre[8] = {
         1,2,5,6,9,10,13,14
    };


     // --- NEW: Add a state variable to track the last sent alert level ---
alert_level_t last_sent_ws2812_level = (alert_level_t)-1; // Init to invalid value

static led_state_t last_sent_led_state{}; 


    // Start ranging
 //   status = vl53l5cx_start_ranging(Dev);

vTaskDelay(pdMS_TO_TICKS(200));

for (;;) {

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal received! Entering vl53l5cx reading loop.");
runCounter = 0;


ESP_LOGD("MUTEX1", "vl53l5cx Waiting for g_i2c_bus mutex...");
if (xSemaphoreTake(g_i2c_bus_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {

vl53l5cx_hasMutex = true;
vTaskDelay(pdMS_TO_TICKS(100));


  ESP_LOGD("MUTEX1", "vl53l5cx Mutex taken.");
        status = vl53l5cx_start_ranging(Dev);

vTaskDelay(pdMS_TO_TICKS(500));

    if (status) {
        ESP_LOGE(TAG, "Failed to start ranging");
         if (vl53l5cx_hasMutex){
        xSemaphoreGive(g_i2c_bus_mutex);
        vl53l5cx_hasMutex = false;
        ESP_LOGD("MUTEX1", "Failed, vl53l5cx Giving g_i2c_bus mutex...");
         }
        vTaskDelete(NULL);
       return;
    }


    while(1) {


#ifdef useAccelerometer


       if (xQueueReceive(mpu_data_queue, &current_orientation, 0) == pdPASS) {
            // New orientation data is now stored in 'current_orientation'
        }

/*
if ( abs(current_orientation.pitch -  last_pitch ) > acc_Move_limit || abs(current_orientation.roll -  last_roll) > acc_Move_limit ) {

 ESP_LOGI("VL53_TASK", "Device is moved (Roll: %.1f). (Pitch: %.1f)  %.1f  %.1f", current_orientation.pitch ,current_orientation.roll, abs(current_orientation.pitch -  last_pitch ), abs(current_orientation.roll -  last_roll)   );
}
// ESP_LOGI("VL53_TASK", "Device is moved (Roll: %.1f). (Pitch: %.1f)  %.1f  %.1f", current_orientation.pitch ,current_orientation.roll, abs(current_orientation.pitch -  last_pitch ), abs(current_orientation.roll -  last_roll)  );

last_pitch = current_orientation.pitch;
last_roll = current_orientation.roll;
*/

/*
            if (current_orientation.pitch > 50.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed RIGHT (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.pitch < -40.0) {
                 ESP_LOGI("VL53_TASK", "Device is pointed LEFT (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }


            if (current_orientation.roll > 140) {
                ESP_LOGI("VL53_TASK", "Device is pointed DOWN (Pitch: %.1f). Adjusting logic.", current_orientation.pitch);
                 // e.g., Treat the top row of the sensor as the "forward" direction
            } else if (current_orientation.roll < 100) {
                ESP_LOGI("VL53_TASK", "Device is pointed UP (Pitch: %.1f). Ignoring floor.", current_orientation.pitch);
                 // e.g., Filter out readings from the bottom rows
            } else {
                 // Device is relatively level, use default logic
            }
*/


#endif


bool topAlert = false;
bool bottomAlert = false;
int centreAlert = 0;



        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (current_time - last_sensor_read >= SENSOR_READ_INTERVAL) {
            last_sensor_read = current_time;


         //   vTaskDelay(pdMS_TO_TICKS(100));
            status = vl53l5cx_check_data_ready(Dev, &isReady);

//ESP_LOGD(TAG, "Sensor task started");
ESP_LOGD(TAG, "Status: %d %d", status, isReady);


            if(isReady) {

gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 

gpio_set_level(RED_PIN_1, 0); 
gpio_set_level(BLUE_PIN_1, 0);  
gpio_set_level(GREEN_PIN_1, 0); 


  not_ready_count = 0; // Reset counter on success
                vTaskDelay(pdMS_TO_TICKS(100));
                vl53l5cx_get_ranging_data(Dev, &Results);

                primary_distance_total = 0;
                uint16_t distance_total = 0;


                for(i = 0; i < 16; i++) {
                    for(int j = 0; j < VL53L5CX_NB_TARGET_PER_ZONE; j++) {
                        uint16_t idx = VL53L5CX_NB_TARGET_PER_ZONE * i + j;

                        uint16_t primary_distance = 0;
                        primary_distance = Results.distance_mm[idx]/10;

// if( runCounter % 10 == 0 ) {
                        distance_total += primary_distance;
                        average_cm = distance_total/(i+1);

 //                       ESP_LOGD(TAG, "average_cm %d %d", i, average_cm);
// }
if ( abs(average_cm - total_average_cm) > 50 ){
    total_average_cm = average_cm; // update total if average_cm changed
}


                        gridAlerts[i] = get_alert_level(primary_distance, total_average_cm);

//printf("grid %d:%d \n", i, gridAlerts[i]);


if ((i % 4) == 0) {

     if( primary_distance <= ALERT_CLOSE_LIMIT   ){
        topAlert = true;
     }

    }



if ((i % 4) == 3) {
 if( primary_distance <= ALERT_CLOSE_LIMIT ){ 
bottomAlert = true;
 }
}


uint8_t remainder = i % 4;
if (remainder == 1 || remainder == 2) {
    // 'i' is in gridCentre.
     primary_distance_total += primary_distance;

if(primary_distance <= ALERT_CLOSE_LIMIT){
    ESP_LOGI(TAG, "centreAlert:%d dist:%d", gridAlerts[i], primary_distance);

    centreAlert ++;
}

}


                    }
                } // for every square

// if( runCounter % 10 == 0 ) {
average_cm = distance_total/16;

if ( abs(average_cm - total_average_cm) > 50 ){
    total_average_cm = average_cm; // update total if average_cm changed
}



 ESP_LOGD(TAG, "average_cm final %d", total_average_cm);
// }
                // Calculate average and update alerts
                int average_distance = primary_distance_total/8;
                alert_level_t alert_level_average = get_alert_level(average_distance, total_average_cm);

//ESP_LOGI(TAG, "average tot:%d avg prim:%d avg all:%d", primary_distance_total, average_distance, total_average_cm);


if (centreAlert >= centreAlert_WarnLevel){
update_beeper_alerts(alert_level_t::ALERT_IMMEDIATE, 1);
ESP_LOGI(TAG, "centreAlert:%d ALERT_IMMEDIATE", centreAlert);
//centreAlert = 0;
} 
/*
else if (centreAlert > 3){
update_beeper_alerts(alert_level_t::ALERT_CLOSE, 1);
ESP_LOGI(TAG, "centreAlert:%d ALERT_CLOSE", centreAlert);

} 
*/
else {
update_beeper_alerts(alert_level_average, 1);
ESP_LOGI(TAG, "centreAlert:%d alert_level_average %d", centreAlert, alert_level_average);
}
centreAlert = 0;



      if (topAlert){
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 1);  
gpio_set_level(GREEN_PIN, 0); 
        }
       if (bottomAlert){
gpio_set_level(RED_PIN, 1); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 
        }
      if (!bottomAlert && !topAlert){
gpio_set_level(RED_PIN, 0); 
gpio_set_level(BLUE_PIN, 0);  
gpio_set_level(GREEN_PIN, 0); 
      }

            // --- Create and populate a single LED state object ---
 //           led_state_t desired_led_state{};
            const alert_config_t *config = &alert_configs[alert_level_average];
    
#ifdef useRGBLed
 // --- NEW: Create and populate a single, simple LED state object ---
    led_state_t desired_led_state{}; // Zero-initialize


    // Map color from the config table
    switch (config->led_color) {
        case LedColor::BLACK:  desired_led_state.r = 0; desired_led_state.g = 0; desired_led_state.b = 0; break;
        case LedColor::RED:    desired_led_state.r = 255; break;
        case LedColor::ORANGE: desired_led_state.r = 255; desired_led_state.g = 140; break;
        case LedColor::YELLOW: desired_led_state.r = 255; desired_led_state.g = 255; break;
        case LedColor::GREEN:  desired_led_state.g = 255; break;
        case LedColor::BLUE:   desired_led_state.b = 255; break;
        case LedColor::WHITE:  desired_led_state.r = 255; desired_led_state.g = 255; desired_led_state.b = 255; break;
        default: break; // For BLACK or OFF, r,g,b will remain 0
    }
    desired_led_state.brightness = config->led_strength_val;

ESP_LOGI(TAG, "LED trying manual light %d %d %d %u", desired_led_state.r, desired_led_state.g, desired_led_state.b, desired_led_state.brightness);
    rgb_led_set_color_with_brightness(desired_led_state.r, desired_led_state.g, desired_led_state.b, 1.0); // 50% brightness red


    // Part 2: Populate the GPIO part of the state
    desired_led_state.top_led_on = topAlert;
    desired_led_state.bottom_led_on = bottomAlert;

     if (desired_led_state != last_sent_led_state) {

 

        ESP_LOGI(TAG, "LED state change detected. Sending update.");
    // Part 3: Send the single, complete state object to the queue
    // We use xQueueOverwrite to ensure the LED task always has the freshest data.

    //  commented out for debug
   xQueueOverwrite(led_state_queue, &desired_led_state);

// Update our record of the last sent state.
                last_sent_led_state = desired_led_state;
     } else {

ESP_LOGI(TAG, "LED state no change detected");
     }
#endif

                // Print grid visualization
                for (int row = 0; row <= 3; row++) {

                        if (row == 0) {
                            printf("\n-\n");
                        }

                    for (int col = 3; col >= 0; col--) {
                        alert_level_t level = gridAlerts[gridLayout[row][col]];
                        
                        switch(level) {
                            

                            case alert_level_t::ALERT_SILENT:
                                printf("%2d", 0);
                                break;
                            case alert_level_t::ALERT_VERYFAR:
                                printf("%2d", 1);
                                break;
                            case alert_level_t::ALERT_FAR:
                                printf("%2d", 2);
                                break;
                            case alert_level_t::ALERT_MEDIUMFAR:
                                printf("%2d", 3);
                                break;
                            case alert_level_t::ALERT_MEDIUM:
                                printf("%2d", 4);
                                break;
                            case alert_level_t::ALERT_CLOSE:
                                printf("%2d", 5);
                                break;
                            case alert_level_t::ALERT_IMMEDIATE:
                                printf("%2d", 6);
                                break;
                        }

                        if (col > 0) {
                            printf(" | ");
                        }
                        
                        if (col == 0) {
                            printf("\n");
                        }
                    }

                   
                }

            

        
              //  loop++;
            } else { // not ready
           not_ready_count++;
            ESP_LOGW(TAG, "Sensor not ready or comms error. Count: %d/%d", not_ready_count, NOT_READY_THRESHOLD);
            

            if (not_ready_count >= NOT_READY_THRESHOLD) {
                ESP_LOGE(TAG, "Not ready threshold reached. Attempting recovery...");
                not_ready_count = 0;
                 if (vl53l5cx_hasMutex){
                xSemaphoreGive(g_i2c_bus_mutex);
                ESP_LOGD("MUTEX1", "vl53l5cx g_i2c_bus Mutex given -> recovery.");
                 }
                vl53l5cx_hasMutex = false;
                // Call the new, comprehensive recovery function
                if (vl53l5cx_recover(Dev)) {
                    // Recovery was successful, reset the counter and continue.
                    
                    vTaskDelay(pdMS_TO_TICKS(50));
                } else {
                    // Catastrophic failure. The sensor couldn't be recovered.
                    // We can either try again after a long delay or stop the task.
                    ESP_LOGE(TAG, "Catastrophic recovery failure. Halting task.");
                    
                    vTaskDelay(pdMS_TO_TICKS(500));
                    //vTaskDelete(NULL); // Or vTaskDelay for a long time...
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(200)); // break, to regain ready state if not > NOT_READY_THRESHOLD
            }

            }
//VL53L5CX_WaitMs(&(Dev->platform), 20);
            // printf("-------------------\n");
        } // SENSOR_READ_INTERVAL passed
           // Periodic Stack Check (e.g., every 10 iterations)
       
        runCounter++;

 
        if( runCounter % 10 == 0 ) {
            minStackBytesRemaining = uxTaskGetStackHighWaterMark(NULL);
      //      ESP_LOGI(TAG, "Sensor Task Stack : %d bytes left", minStackBytesRemaining);
            if( minStackBytesRemaining < 100 ){
                ESP_LOGW(TAG, "Sensor Task Stack Warning: %d bytes left", minStackBytesRemaining);
            }
        }
        
     if (vl53l5cx_hasMutex){
xSemaphoreGive(g_i2c_bus_mutex);
ESP_LOGD("MUTEX1", "vl53l5cx Giving g_i2c_bus mutex...");
     }
vl53l5cx_hasMutex = false;

        // Small delay to prevent watchdog issues and allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(50));

//esp_task_wdt_feed();

               // --- CRITICAL PART: Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal received! vl53l5cx Exiting loop and going back to sleep.");
                    break; // Exit the inner "running" loop
                }

                
     } // while 1 loop end

if (vl53l5cx_hasMutex){
   xSemaphoreGive(g_i2c_bus_mutex);
      ESP_LOGD("MUTEX1", "g_i2c_bus Mutex given.");
}
 vl53l5cx_hasMutex = false;
    } //  got mutex
        else {
            vl53l5cx_hasMutex = false;
        ESP_LOGW(TAG, "No mutex available");
    //    xSemaphoreGive(g_i2c_bus_mutex);
        }


 


    }  // got semaphore  signal  



} // waiting for g_binary_semaphore

    // Cleanup (though this won't be reached in normal operation)
    vl53l5cx_stop_ranging(Dev);
  
    set_beeper_tone(0, false);
    vTaskDelete(NULL);
}
#endif



#ifdef usePinPolling
// polling gpio 
// How often (ms) to poll
static constexpr TickType_t POLL_PERIOD_MS = 500;
// Prototype for your action
void pin_triggered_function(gpio_num_t pin);


using gpio_cb_t = void(*)(gpio_num_t pin);

// Maximum GPIO number on the S3
static constexpr int MAX_GPIO = 40;
#endif


//#define CAM_PIN        GPIO_NUM_40
//gpio_set_direction(CAM_PIN, GPIO_MODE_INPUT);

///// polling gpio version //////
// Task that polls one pin

 // static u_int8_t AlertPin_value = 0;
//static pin_values_t pinValues[5] = {5, 7, 8, 9, 40}; 


// Configuration
#define DEBOUNCE_MS 25
#define MAX_BUTTONS 4       // Reduce if fewer buttons needed
#define BUTTON_TASK_STACK 2048
#define BUTTON_TASK_PRIORITY 1  // Low priority since buttons are rarely used

typedef struct {
    gpio_num_t pin;
    int last_state;  // Changed from bool to int (gpio_get_level returns int)
    uint32_t last_change_time;
} button_t;

static button_t buttons[MAX_BUTTONS];
static size_t num_buttons = 0;
static QueueHandle_t button_event_queue = NULL;

typedef struct {
    gpio_num_t pin;
    int new_state;   // Changed from bool to int
} button_event_t;

void button_task(void *arg) {
    button_event_t event;

      ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
      

    while(1) {

           ESP_ERROR_CHECK(esp_task_wdt_reset());

        
        for (int i = 0; i < num_buttons; i++) {
            int current = gpio_get_level(buttons[i].pin); // gpio_get_level returns int
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            if (current != buttons[i].last_state) {
                if ((now - buttons[i].last_change_time) > DEBOUNCE_MS) {
                    event.pin = buttons[i].pin;
                    event.new_state = current;
                    xQueueSend(button_event_queue, &event, 0);
                    
                    buttons[i].last_state = current;
                    buttons[i].last_change_time = now;
                }
            } else {
                buttons[i].last_change_time = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t button_init(gpio_num_t gpio_num, gpio_pull_mode_t pull_mode) {



    if (num_buttons >= MAX_BUTTONS) return ESP_FAIL;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = (pull_mode == GPIO_PULLUP_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = (pull_mode == GPIO_PULLDOWN_ONLY || pull_mode == GPIO_PULLUP_PULLDOWN) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    buttons[num_buttons] = (button_t){
        .pin = gpio_num,
        .last_state = gpio_get_level(gpio_num), // Returns int (0 or 1)
        .last_change_time = 0
    };
    num_buttons++;
ESP_LOGI("BUTTON", "inited button pin %d", (int) gpio_num);

    return ESP_OK;
}

void button_system_init() {
    button_event_queue = xQueueCreate(5, sizeof(button_event_t));
    xTaskCreate(button_task, "button_task", BUTTON_TASK_STACK, NULL, BUTTON_TASK_PRIORITY, NULL);
}

bool get_button_event(button_event_t *event, uint32_t timeout_ms) {

//ESP_LOGI("BUTTON", "Got button event");

    return xQueueReceive(button_event_queue, event, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
   // return false;
}


#ifdef usePinPolling
static void poll_pin_task(void* arg)
{
    gpio_num_t pin = (gpio_num_t)(uintptr_t)arg;
   // int pinArrayPlace = find(pinValues,sizeof(pinValues),(int) pin);
    // int pinArrayPlace = find(pinValues,5,(int) pin);

//ESP_LOGI("POLLINIT", "Init for Pin %d init at place %d ", pin, pinArrayPlace  );
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
          

    for (;;) {
        int level = gpio_get_level(pin);
      //  int trigger_level = arg.pin;
      int trigger_level = 1;

//int pinArrayPlace = find(pinValues,sizeof(pinValues),(int) pin);
//pinArrayPlace = find(pinValues,5,(int) pin);
//int currentvalue = pinValues[pinArrayPlace];
// pinValues[pinArrayPlace];

           ESP_ERROR_CHECK(esp_task_wdt_reset());


ESP_LOGI("POLL", "Pin %d has level %d ", pin,level);
//pinValues[pinArrayPalce]

        if (level == trigger_level ) { //&& currentvalue != level
            ESP_LOGI("POLL", "Pin %d is %d → triggering, value now is %d", pin, trigger_level );
            pin_triggered_function(pin);
            // wait for it to go back HIGH before retriggering
            while (gpio_get_level(pin) != trigger_level) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
          //  pinValues[pinArrayPlace] = level;


           // ESP_LOGI("POLL", "setting %d at %d to %d", pin, pinArrayPlace, level);
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
        // vTaskDelay(pdMS_TO_TICKS(500));
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
        4048,
        (void*)(uintptr_t)pin,
        tskIDLE_PRIORITY + 1,
        nullptr
    );
    return (ok == pdPASS) ? ESP_OK : ESP_FAIL;
}
#endif


#ifdef usePinPolling
///// polling gpio version //////
// Example “action” when pin goes low
void pin_triggered_function(gpio_num_t pin)
{
#ifdef useXiaoCam
    if ( (uint8_t) pin == CAM_PIN){
      //  save_nvs(nvs_var_serial, 0);
      // person detected
ESP_LOGI("CAM", "Person detected %d", pin);
blink_task_init(GREEN_PIN, 5, 80);
    }
#endif

/*
   if ( (uint8_t) pin == ALERT_TYPE_PIN){

//AlertPin_value = 1;
      //  save_nvs(nvs_var_serial, 0);
      // person detected
ESP_LOGI("AlertPin", "Alertswitch detected %d", pin);
//blink_task_init(GREEN_PIN, 5, 80);
useVibratorAlerts = !useVibratorAlerts;

if(useVibratorAlerts){
useAudioAlerts = false ;
   }   else {
useAudioAlerts = true ;
    }
 vTaskDelay(pdMS_TO_TICKS(2000));
}
*/
    // …your code here…
    ESP_LOGI("ACTION", "Handling event from pin %d", pin);
}

#endif


 #ifdef useUltrasound



 void ultrasonic_ranger(void *pvParameters) {

    // ultrasound_mutex
//ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
   

    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO,
    };
       ultrasonic_sensor_t sensor2 = {
        .trigger_pin = TRIGGER_GPIO_2,
        .echo_pin = ECHO_GPIO_2,
    };

        ultrasonic_init(&sensor);
        ultrasonic_init(&sensor2);

uint32_t last_update_time = xTaskGetTickCount();
static int ultrasonic_update_period = 5;
/*
  ultrasonic_sensor_t sensor = {
    .trigger_pin, //!< GPIO output pin for trigger
    .echo_pin,    //!< GPIO input pin for echo
    .minDistance = 5,
    .maxDistance = 300,
    .maxRetry = 3,
    .pulseTuning = 200,
    .debug = false,
} 
*/


for (;;) {

//    esp_task_wdt_delete(NULL); 
   
 // ESP_ERROR_CHECK(esp_task_wdt_reset());

            uint32_t notification_value = 0;

        // 1. Wait indefinitely for any notification (e.g., the START signal)
        // This call will block until xTaskNotify is called on this task.
        xTaskNotifyWait(0,           /* Don't clear any bits on entry */
                        ULONG_MAX,   /* Clear all bits on exit */
                        &notification_value, /* Stores the notification value */
                        portMAX_DELAY);      /* Block forever */

        // 2. Check if the START signal was received
        if (notification_value & START_BIT) {
            ESP_LOGI(TAG, "START signal for ultrasonic reading loop 1.");

//static int sensornumber = 0;
//ultrasonic_sensor_t active_sensor = sensor;
     uint32_t distance = 0;
     uint32_t distance2 = 0;

float temp = 25.0;
uint32_t  min_time_between_ultrasound = 50;


    while (true)
    {
   
ESP_ERROR_CHECK(esp_task_wdt_reset());


  if (xSemaphoreTake(ultrasound_mutex, portMAX_DELAY) == pdTRUE) {
 ESP_LOGD(TAG, "ultrasound Got mutex .");
            alert_level_t alert_level = ALERT_VERYFAR;
    ///    alert_level_t alert_level_2 = ALERT_VERYFAR;



//if (current_time - last_update_time > ultrasonic_update_period ) {
// last_update_time = current_time;


uint32_t u1_read_time = xTaskGetTickCount();
            esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);
     //   esp_err_t res = ultrasonic_measure_cm_temp_compensated(&sensor, MAX_DISTANCE_CM, &distance, temp);
 

     if (res != ESP_OK)
       {
          printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping ultrasonic sensor 1 (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout ultrasonic sensor 1 (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout ultrasonic sensor 1\n");
                    alert_level = get_alert_level( 500, 200);
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else {

  
         printf("Distance from sensor %d: %lu\n", 1, distance);
            alert_level = get_alert_level( distance, 200);
      printf("sens1 get_alert_level %lu\n", distance);



//ESP_LOGI(TAG, "update_beeper_alerts: %d ",  alert_level);
update_beeper_alerts(alert_level, 1);
} // no error reading

//play_sine_tone(200, 1200);
              
//ESP_LOGI(TAG, "alert_level dist: %lu ",  distance);
//  if (distance2 <= distance) {
  //      alert_level_2 = get_alert_level( distance2, 200);
   //     update_beeper_alerts(alert_level_2, 2);
//  } else {
  //      alert_level = get_alert_level( distance, 200);
   //     update_beeper_alerts(alert_level, 1);
//  }


vTaskDelay(pdMS_TO_TICKS(min_time_between_ultrasound)); // wait for signal from sensor1 to die out


////// sens 2
//     uint32_t distance2 = 0;
   alert_level_t alert_level_2 = ALERT_VERYFAR;



//    ultrasonic_init(&sensor2);


uint32_t u2_read_time = xTaskGetTickCount();

/*
while ( (u2_read_time - u1_read_time) < min_time_between_ultrasound){
vTaskDelay(pdMS_TO_TICKS(20));
u2_read_time = xTaskGetTickCount();
 ESP_LOGI(TAG, "waiting for %d - %d > %d", u2_read_time, u1_read_time, min_time_between_ultrasound );
}
*/
        res = ultrasonic_measure_cm(&sensor2, MAX_DISTANCE_CM, &distance2);
     //  res = ultrasonic_measure_cm_temp_compensated(&sensor2, MAX_DISTANCE_CM, &distance2, temp);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping ultrasonic sensor 2 (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout ultrasonic sensor 2 (no device found )\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout ultrasonic sensor 2 \n");
                    alert_level_2 = get_alert_level( 500, 200);
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else {
           // printf("Distance from sensors 1:%lu 2:%lu\n",  distance, distance2);
         //   alert_level_2 = get_alert_level( distance2, 200);
         
//       ultraSensorDistance1 = distance;
//       ultraSensorDistance2 = distance2;

 printf("Distance from sensor %d: %lu\n", 2, distance2);

 //   printf("Distance from sensors 1:%lu 2:%lu\n",  distance, distance2);
                //  printf("Distance from sensors 2:%lu\n",  distance2);
//ESP_LOGI(TAG, "alert_level dist: %lu ",  distance);
//  if (distance2 <= distance) {

float adjusted_distance = 5;
float distance2f = distance2;
if(distance2f + sensor2_diff <= 0) {
    adjusted_distance = 5;
} else {
adjusted_distance = distance2f + sensor2_diff ;
}
// printf("sending adjusted_distance:%f \n", adjusted_distance);


        alert_level_2 = get_alert_level( adjusted_distance , 200);
        update_beeper_alerts(alert_level_2, 2);
//  } else {
     //   alert_level = get_alert_level( distance, 200);
      //  update_beeper_alerts(alert_level, 1);
//  }

 } // no error reading

xSemaphoreGive(ultrasound_mutex);
 ESP_LOGD(TAG, "ultrasound gave mutex .");
//vTaskDelay(pdMS_TO_TICKS(100));
       // vTaskDelay(pdMS_TO_TICKS(500));

        
//            } //update time


} // // got ultrasound_mutex 
else {
      ESP_LOGI(TAG, "No mutex for ultra 1."); 
}

               // --- Check for a STOP signal ---
                uint32_t stop_signal_value = 0;
                // Use a zero timeout to check instantly without blocking
                xTaskNotifyWait(0, ULONG_MAX, &stop_signal_value, 0);

                // 3. If a STOP signal was sent, break the inner loop
                if (stop_signal_value & STOP_BIT) {
                    ESP_LOGI(TAG, "STOP signal for ultrasound loop.");
                    break; // Exit the inner "running" loop
                }
vTaskDelay(pdMS_TO_TICKS(100));
    }/// loop while true
    


} // start semaphore

vTaskDelay(pdMS_TO_TICKS(100));
//esp_task_wdt_add(NULL);

} // outer loop waiting for semaphore

}
#endif



void find_reset_reason( int resetreason){

   // SW_CPU_RESET

    if (resetreason == 8) {
        ESP_LOGI(TAG, "Reset ESP_RST_DEEPSLEEP");
     //       break;
    
    //  return();
    }
    
              if (resetreason == 3) {
                 ESP_LOGI(TAG, "Reset ESP_RST_SW");
        //       break;
     
           //  return();
          }
     
              if (resetreason == 4 || resetreason == 14) {
                 ESP_LOGI(TAG, "Reset ESP_RST_PANIC or ESP_RST_CPU_LOCKUP");
       //       break;
     
           //  return();
          }
     
              if (resetreason == 5) {
                 ESP_LOGI(TAG, "Reset ESP_RST_INT_WDT");
       //       break;
     
           //  return();
          }
     
              if (resetreason == 9 || resetreason == 14) {
                 ESP_LOGI(TAG, "Reset ESP_RST_BROWNOUT or ESP_RST_PWR_GLITCH");
            //       break;
     
           //  return();
          }
     
          if (resetreason == 6) {
             ESP_LOGI(TAG, "Reset ESP_RST_TASK_WDT");
      
       //  return();
      }
     
     
  

  }
 
void initialize_watchdog() {
    // Check if WDT is already initialized
    if (esp_task_wdt_status(NULL) == ESP_ERR_NOT_FOUND) {
        esp_task_wdt_config_t wdt_config = {
            .timeout_ms = 5000,
            .idle_core_mask = (1 << 0) | (1 << 1),
            .trigger_panic = true
        };
        ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_config));
        ESP_LOGI("WDT", "Watchdog initialized");
    } else {
        ESP_LOGI("WDT", "Watchdog already running");
    }
}

extern "C" void app_main(void)
{

    ESP_LOGI("MAIN", "Starting up");


    // 1. Basic system initialization first
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

 //initialize_watchdog();
// already auto inited in config


 #ifdef useGPS_2  
    nmea_example_init_interface();
   read_and_parse_nmea();
#endif

gpio_set_direction(RED_PIN, GPIO_MODE_OUTPUT);
gpio_set_direction(BLUE_PIN, GPIO_MODE_OUTPUT);
gpio_set_direction(GREEN_PIN, GPIO_MODE_OUTPUT);

gpio_set_direction(RED_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_direction(BLUE_PIN_1, GPIO_MODE_OUTPUT);
gpio_set_direction(GREEN_PIN_1, GPIO_MODE_OUTPUT);




/////// i2c init //////

initialize_i2c();
xTaskCreate(i2c_task, "i2c_task", 4096, NULL, 10, NULL);
/*
    i2c_port_t i2c_port = I2C_NUM_1;


    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = i2c_port,
        .sda_io_num = GPIO_NUM_1,
        .scl_io_num = GPIO_NUM_2,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags {
            .enable_internal_pullup = false,
            .allow_pd = false,
        }
    };

   i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
*/

/*
    g_i2c_bus_mutex = xSemaphoreCreateMutex();
    if (g_i2c_bus_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create I2C bus mutex!");
      //  return;
    }
      */
/////// end i2c init /////////

 #ifdef useLuna

 //  xTaskCreate(tf_luna_task, "tf_luna", 4096, bus_handle, 5, NULL);

  /* 
    if (tf_luna_init() != ESP_OK) {
        ESP_LOGE("MAIN", "TF-Luna init failed");
        return;
    } else {
          ESP_LOGI("TF_LUNA", "Luna inited");
    }
*/
#endif
 //#ifndef useLuna

//scan_i2c_devices();


 #ifdef useGPS    
            uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create UART  mutex!");
        //return;
    }
       
#endif

 #ifdef useUltrasound    
            ultrasound_mutex = xSemaphoreCreateMutex();
    if (ultrasound_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create Ultrasound mutex!");
        //return;
    }
     
#endif


#ifdef useAccelerometer
        mpu_data_queue = xQueueCreate(5, sizeof(mpu_data_t));
#endif

#ifdef useRGBLed;
led_state_queue = xQueueCreate(1, sizeof(led_state_t)); // Correct queue
ESP_LOGI("MAIN", "Mutex and Queues created.");
#endif

#ifdef useGPS
GPS_init();

xTaskCreatePinnedToCore(
                    gps_task,   /* Function to implement the task */
                    "gps_task", /* Name of the task */
                    1024 * 3,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    4,          /* Priority of the task */
                    &xHandle_GPS,       /* Task handle. */
                    0);  /* Core where the task should run */
 
 ESP_LOGI("MAIN", "GPS started");

#endif

 //   i2c_scan_devices() ;
 //  vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for the sensor to stabilize


#ifdef useAccelerometer

// --- MPU9250 Initialization ---
    i2c_master_dev_handle_t mpu9250_dev_handle;
    i2c_device_config_t mpu9250_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_I2C_ADDRESS, // 0x68
        .scl_speed_hz = 100000,
    };

     /*
      esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
            vSemaphoreDelete(i2c_mutex);
            return ret;
        }
      */  

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu9250_dev_cfg, &mpu9250_dev_handle));
    ESP_LOGI("I2C", "Device MPU9250 added to bus");
    
    // IMPORTANT: Wake up the MPU9250 from sleep mode
    ESP_LOGI("MPU9250", "Waking up aceelerometer sensor...");



    esp_err_t ret3 = i2c_master_transmit(
        mpu9250_dev_handle, 
        (const uint8_t[]){ MPU9250_PWR_MGMT_1_REG_ADDR, 0x00 }, // Write 0 to PWR_MGMT_1
        2, 
        pdMS_TO_TICKS(100)
    );
    if (ret3 != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mpu9250_dev_handle");
       // return;
    } else {
    ESP_LOGI(TAG, "MPU9250 Initialized.");
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for the sensor to stabilize

#endif



    ESP_LOGI("MAIN", "Initialization complete. Tasks are running.");

 

   // Initialize piezo beeper
    ret = init_piezo_beeper(); //esp_err_t 
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize piezo beeper");
        return;
    }
   ESP_LOGI(TAG, "initialized piezo beeper");


#ifdef useXiaoCam
//#define CAM_PIN        GPIO_NUM_40
gpio_set_direction(CAM_PIN, GPIO_MODE_INPUT);

ESP_ERROR_CHECK(init_polled_input(CAM_PIN, false));
#endif

//gpio_set_direction(ALERT_TYPE_PIN, GPIO_MODE_INPUT);
//ESP_ERROR_CHECK(init_polled_input(ALERT_TYPE_PIN, false));



#ifdef useVl53l5cx

 // Create an I2C device handle for the VL53L0X
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = VL53L0X_DEFAULT_I2C_ADDR, 
      .scl_speed_hz = 100000,
};
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_vl53l0x.i2c_dev));
  ESP_LOGI(TAG, "VL53L0X device added to I2C bus");
 
// Initialize the sensor via its descriptor
  ESP_ERROR_CHECK(vl53l0x_init(&dev_vl53l0x));
  ESP_LOGI(TAG, "VL53L0X Initialized");


  /*
// --- 1. Declare necessary variables ---
uint8_t status, isAlive;
VL53L5CX_Configuration Dev; // Your main driver struct

// --- 2. Define the I2C device configuration for the VL53L5CX ---
i2c_device_config_t vl53_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    // Use the raw 7-bit address. The default is 0x29.
    // The driver handles the R/W bit, so no shifting is needed.
    .device_address = 0x29, 
    .scl_speed_hz = VL53L5CX_MAX_CLK_SPEED,
};


// --- 3. Add the device to the bus. This creates the handle. ---
// The handle is stored inside your Dev struct, ready for the driver to use.
ESP_LOGI(TAG, "Adding VL53L5CX sensor to I2C bus...");
i2c_master_bus_add_device(bus_handle, &vl53_dev_cfg, &Dev.platform.handle);

// --- 4. Now that communication is possible, perform hardware reset and init ---
ESP_LOGI(TAG, "Resetting VL53L5CX sensor...");

Dev.platform.reset_gpio = GPIO_NUM_5;
VL53L5CX_Reset_Sensor(&(Dev.platform));

// --- 5. Check if the sensor is alive (this uses the handle created in step 3) ---
ESP_LOGI(TAG, "Checking if VL53L5CX is alive...");
status = vl53l5cx_is_alive(&Dev, &isAlive);
if (!isAlive || status) {
    ESP_LOGE(TAG, "VL53L5CX not detected at address 0x29! Check wiring and address.");
    return; // Stop here if sensor is not found
}
ESP_LOGI(TAG, "VL53L5CX is alive!");

// --- 6. Initialize the sensor (this also uses the handle created in step 3) ---
ESP_LOGI(TAG, "Initializing VL53L5CX ULD...");
status = vl53l5cx_init(&Dev);
if (status) {
    ESP_LOGE(TAG, "VL53L5CX ULD Loading failed, error code: %d", status);
    return;
}

ESP_LOGI(TAG, "VL53L5CX ULD ready! (Version: %s)", VL53L5CX_API_REVISION);

    // Prepare task parameters
    static sensor_task_params_t task_params = {
        .dev = &Dev,
        .gridLayout = gridLayout,
        .gridAlerts = gridAlerts
    };
    */
#endif

 // --- 5. Create and Launch All Tasks ---
    ESP_LOGI("MAIN", "Creating tasks...");

#ifdef useRGBLed
    ESP_LOGI("MAIN", "Creating task for RGB LED");
    // Create the LED task. It's now safe to start.
    xTaskCreate(led_control_task, "LED Control", 4096, NULL, 4, NULL);
#endif


#ifdef useUltrasound
 // xTaskCreate(hcsr04_task, "HSRC04 task", 2048, NULL, 4, NULL);
 xTaskCreatePinnedToCore(
                    ultrasonic_ranger,   /* Function to implement the task */
                    "ultrasonic_ranger", /* Name of the task */
                    1024 * 3,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    5,          /* Priority of the task */
                    &xHandle_ultrasonic,       /* Task handle. */
                    0);  /* Core where the task should run */

// start ultrasound immediately
 xTaskNotify( xHandle_ultrasonic, START_BIT, eSetBits);

#endif
//ultrasonic_ranger_2
#ifdef useUltrasound_2
xTaskCreatePinnedToCore(
                    ultrasonic_ranger_2,   /* Function to implement the task */
                    "ultrasonic_ranger_2", /* Name of the task */
                    8192,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    5,          /* Priority of the task */
                    &xHandle_ultrasonic_2,       /* Task handle. */
                    0);  /* Core where the task should run */
  //    xTaskCreate(ultrasonic_ranger, "ultrasonic_ranger", 8192 , NULL, 5, &xHandle_ultrasonic);
#endif

#ifdef useUartA02YYUW
    init_uart();
    xTaskCreate(uart_task, "UART Task", 4096, NULL, 5, NULL);
#endif

#ifdef useUart
 init_uart();
 #endif


    // Create sensor and data processing tasks
#ifdef useAccelerometer
ESP_LOGI("MAIN", "Creating task for Accelerometer");

 
xTaskCreate(mpu9250_reader_task, "MPU Reader", 1024 * 3, mpu9250_dev_handle, 3, &xHandle_mpu);
xTaskCreate(data_processor_task, "Data Processor", 1024 * 3, NULL, 3, &xHandle_mpu_processor);
    #endif

// Static storage for task parameters
//static blink_task_params_t task_params;
#ifdef useVl53l5cx
    xTaskCreate(vl53l5cx_reader_task, "VL53L5CX Reader", 8192, &task_params, 5, &xHandle_vl53l5cx);
#endif

//vl53l5cx_running = false;
///// vibration

//for (int i = 0; i < SAMPLE_CNT; ++i)
 #ifdef useVibrator
 example_ledc_init();
uint32_t duty = 8000;
int count = 0;

       while (count < 10)
    {
    
   // LEDC_DUTY = 2000;

    //#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
    // Set duty to 50%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
 vTaskDelay(1000 / portTICK_PERIOD_MS);
 duty += 100;
        count++;
    }
 #endif

#ifdef useLunaX




    tf_luna_queue = xQueueCreate(5, sizeof(tf_luna_data_t));
    if (!tf_luna_queue) {
        ESP_LOGE("TF_LUNA", "Queue creation failed");
     //   return ESP_FAIL;
    }

    // Add TF-Luna device to I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TF_LUNA_ADDR, //0x10
        .scl_speed_hz = 100000,
    };
    
    esp_err_t ret2 = i2c_master_bus_add_device(bus_handle, &dev_cfg, &tf_luna_handle);
    if (ret2 != ESP_OK) {
        ESP_LOGE("TF_LUNA", "Device init failed: 0x%x", ret2);
        vQueueDelete(tf_luna_queue);
     //   return ret;
    }
ESP_LOGI("TF_LUNA", "Device added i2c_master_bus_add_device");

     vTaskDelay(pdMS_TO_TICKS(200)); // Wait for the sensor to stabilize



    // Create task

    xTaskCreate(tf_luna_task, "tf_luna_task", 8192, NULL, TF_LUNA_TASK_PRIO, NULL);



    BaseType_t task_ret = xTaskCreate(
        tf_luna_task,
        "tf_luna_task",
        TF_LUNA_TASK_STACK,
        NULL,
        TF_LUNA_TASK_PRIO,
        NULL
    );
  
    if (task_ret != pdPASS) {
        ESP_LOGE("TF_LUNA", "Task creation failed");
        i2c_master_bus_rm_device(tf_luna_handle);
        vQueueDelete(tf_luna_queue);
       // return ESP_FAIL;
    }
    





//ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &mpu9250_dev_cfg, &mpu9250_dev_handle));
 //esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &tf_luna_handle);
    // Initialize TF-Luna system
 //   if (tf_luna_system_init() != ESP_OK) {
  //      ESP_LOGE("MAIN", "Failed to initialize TF-Luna");
 //       return;
 //   }

tf_luna_data_t latest;

#endif


#ifdef useAudio
initPWMaudio2 ();
/*
if (initPwmAudio () ){
     ESP_LOGE(TAG, "initPwmAudio success");
}
*/


ESP_LOGI("MAIN", "initAudioSystem done");


  audioQueue = xQueueCreate(1, sizeof(alert_level_t));
  if (audioQueue == NULL) {
    ESP_LOGE(TAG, "Error creating the audioQueue  queue");
    return;
  }

 // configure_sdm_audio();
 // xTaskCreate(eventGeneratorTask, "EventGeneratorTask", 4096, NULL, 5, &eventGeneratorTaskHandle);
 // xTaskCreate(audioTask, "AudioTask", 4096, NULL, 5, &audioTaskHandle);

audio_mutex = xSemaphoreCreateMutex();
    if (audio_mutex == NULL) {
        ESP_LOGE("MAIN", "Fatal: Failed to create audio_mutex mutex!");
        return;
    }

 xTaskCreatePinnedToCore(
                    audioTask,   /* Function to implement the task */
                    "AudioTask", /* Name of the task */
                    1024 * 3,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    4,          /* Priority of the task */
                    &audioTaskHandle,       /* Task handle. */
                    1);  /* Core where the task should run */

//xTaskCreate(audioTask, "AudioTask", 4096, NULL, 5, &audioTaskHandle);


 ESP_LOGI("MAIN", "Audio started");
//playWarningToneAsync(ALERT_CLOSE, 200);
//playWarningToneAsync(ALERT_IMMEDIATE, 200);
//playWarningToneAsync(ALERT_IMMEDIATE, 500);
#endif

// blink_task_init(5, 500); // Blink 5 times with 500 ms interval

//button_init(GPIO_NUM_40, GPIO_PULLUP_ONLY);
    button_init(ALERT_TYPE_PIN, GPIO_PULLUP_ONLY); //pin 39

    button_system_init();
    //button_system_init();

    button_event_t event;

esp_err_t t_err = esp_timer_init();
 if (t_err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to esp_timer_init: %s", esp_err_to_name(t_err));
        }


  static unsigned long start_time1 = esp_timer_get_time();
  static unsigned long start_time2 = esp_timer_get_time();

  /*
  if(millis() - start_time >= 10*1000) {
    do_some_work1();
    start_time1 = millis();
  }

  if(millis() - start_time >= 20*1000) {
    do_some_work2();
    start_time2 = millis();
  }
*/


        while(1) {


uint32_t current_time =  esp_timer_get_time();



 if(current_time - start_time1 >= 500) {
 start_time1 =  esp_timer_get_time();

        if (get_button_event(&event, 100)) { // 100ms timeout
            ESP_LOGI("BUTTON", "Pin %d: %s", event.pin, event.new_state ? "PRESSED" : "RELEASED");


 //           typedef struct {
 //   uint8_t pin;
 //  bool new_state;
//} button_event_t;


if (event.new_state == true && event.pin == ALERT_TYPE_PIN){
useVibratorAlerts = !useVibratorAlerts;

if(useVibratorAlerts){
useAudioAlerts = false ;
   }   else {
useAudioAlerts = true ;
    }
}

  } 
            // Handle event
        } // start_time


#ifdef useLunaX

 tf_luna_data_t data;

if(current_time - start_time2 >= 700) {
     start_time2 = esp_timer_get_time();


 //tf_luna_read_with_retry(&data);
 //ESP_LOGI("TF-LUNA", "Distance: %dcm | Strength: %d | Temp: %.1f°C", data.distance, data.strength, data.temperature);

vTaskDelay(pdMS_TO_TICKS(200));

        if (xQueueReceive(tf_luna_queue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI("TF-LUNA", "Distance: %dcm | Strength: %d | Temp: %.1f°C",
                    data.distance, data.strength, data.temperature);
        }
/*
        if (tf_luna_get_latest(&latest)) {
            ESP_LOGI("MAIN", "Distance: %dcm | Strength: %d | Temp: %.1fC",
                    latest.distance, latest.strength, latest.temperature);
        }
       // vTaskDelay(pdMS_TO_TICKS(100));
       */
    }
 #endif


 //vTaskDelay(pdMS_TO_TICKS(100));

    }



}
