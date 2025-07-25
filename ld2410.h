#ifndef LD2410_H
#define LD2410_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

//#include "esp_idf_lib_helpers.h"

#define LD2410_UART_NUM         UART_NUM_1
#define LD2410_UART_BAUD_RATE   115200
#define LD2410_BUF_SIZE         1024

// Frame constants
#define LD2410_FRAME_HEADER     0xF4F3F2F1
#define LD2410_FRAME_FOOTER     0xF8F7F6F5
#define LD2410_CMD_FRAME_HEADER 0xFDFCFBFA
#define LD2410_CMD_FRAME_FOOTER 0x04030201

// Data types
#define LD2410_DATA_FRAME       0x01
#define LD2410_CMD_FRAME        0x02

// Command constants
#define LD2410_CMD_ENABLE_CONF      0xFF
#define LD2410_CMD_DISABLE_CONF     0xFE
#define LD2410_CMD_READ_PARAM       0x61
#define LD2410_CMD_WRITE_PARAM      0x60
#define LD2410_CMD_ENABLE_ENG       0x62
#define LD2410_CMD_DISABLE_ENG      0x63
#define LD2410_CMD_SET_GATE_SENS    0x64
#define LD2410_CMD_READ_VERSION     0xA0
#define LD2410_CMD_SET_BAUD         0xA1
#define LD2410_CMD_FACTORY_RESET    0xA2
#define LD2410_CMD_RESTART          0xA3
#define LD2410_CMD_SET_BLUETOOTH    0xA4
#define LD2410_CMD_SET_LIGHT_FUNC   0xA5
#define LD2410_CMD_QUERY_DISTANCE_RESOLUTION  0xAB
#define LD2410_CMD_SET_DISTANCE_RESOLUTION    0xAA

#define MAX_TRUSTED_JUMP_CM  100  // Reject jumps >80cm between samples
#define MIN_CONFIDENCE_ENERGY 15 // Minimum energy % to consider valid

#define DEFAULT_WINDOW_SIZE     15  // Baseline for new buffers


#define MIN(a, b) ({ \
    typeof(a) _a = (a); \
    typeof(b) _b = (b); \
    _a < _b ? _a : _b; \
})

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

// Simple clamp macro
#define CLAMP(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))


#define LD2410_RX_BUFFER_SIZE 256
#define LD2410_MAX_FRAME 256
#define LD2410_SAFE_READ_LEN  MIN(128, LD2410_MAX_FRAME-10)  // Operational buffer


// Gate definitions
#define LD2410_MAX_GATES        9
#define MAX_WINDOW_SIZE         50
#define OUTLIER_THRESHOLD 500  // cm (adjust based on your observed noise)
// Near range (0-2m): tighter filtering
#define NEAR_NOISE_THRESHOLD 50  // cm 
// Far range (>2m): looser filtering  
#define FAR_NOISE_THRESHOLD 150   // cm

// Target states
typedef enum {
    LD2410_TARGET_NONE = 0,
    LD2410_TARGET_MOVING = 1,
    LD2410_TARGET_STATIONARY = 2,
    LD2410_TARGET_MOVING_AND_STATIONARY = 3
} ld2410_target_state_t;

typedef enum {
    DISTANCE_RESOLUTION_0_75 = 0x00,  // Default: 0.75m resolution  
    DISTANCE_RESOLUTION_0_2  = 0x01   // High precision: 0.2m resolution
} distance_resolution_t;


typedef struct {
    uint16_t *buffer;
    uint32_t sum;
    uint8_t window_size;
    uint8_t current_index;
    uint8_t count;
    bool is_full;
    uint16_t average;
    uint16_t last_value;
    int16_t trend;
} moving_average_t;

typedef struct {
    ld2410_target_state_t target_state;
    uint16_t moving_target_distance;     // cm
    uint8_t moving_target_energy;        // 0-100
    uint16_t stationary_target_distance; // cm
    uint8_t stationary_target_energy;    // 0-100
    uint16_t detection_distance;         // cm
    
    // Raw values for debugging
    uint16_t raw_moving_target_distance;
    uint16_t raw_stationary_target_distance;
    uint16_t raw_detection_distance;
} ld2410_target_data_t;

// Engineering mode data (all gate energies)
typedef struct {
    uint8_t max_moving_gate;
    uint8_t max_stationary_gate;
    uint8_t moving_energy[LD2410_MAX_GATES];
    uint8_t stationary_energy[LD2410_MAX_GATES];
    uint16_t light_value;
    uint8_t out_pin_level;
} ld2410_engineering_data_t;

// Configuration structure
typedef struct {
    uint8_t max_move_distance_gate;     // 0-8
    uint8_t max_still_distance_gate;    // 0-8
    uint8_t move_thresholds[LD2410_MAX_GATES];   // 0-100 for each gate
    uint8_t still_thresholds[LD2410_MAX_GATES];  // 0-100 for each gate
    uint16_t timeout;                   // seconds
    uint16_t light_threshold;           // 0-255
    bool light_function_enabled;
    bool bluetooth_enabled;
    bool engineering_mode;
    distance_resolution_t distance_resolution;  // NEW: distance resolution
 
} ld2410_config_t;

typedef struct {
    bool is_connected;
    ld2410_target_data_t target_data;
    ld2410_engineering_data_t engineering_data;
    ld2410_config_t config;
    
    // Moving averages
    moving_average_t moving_distance_avg;
    moving_average_t stationary_distance_avg;
    moving_average_t detection_distance_avg;
    
    // Binary presence detection
    gpio_num_t presence_pin;
    bool presence_detected;
    bool presence_pin_enabled;
    uint32_t presence_high_count;
    uint32_t presence_low_count;
    uint32_t presence_total_reads;
    
    // Frame parsing state
    uint8_t rx_buffer[LD2410_RX_BUFFER_SIZE];
    uint16_t rx_buffer_position;  // Now matches buffer max size

    uint32_t last_periodic_millis;
    
    // Statistics
    uint32_t total_frames_received;
    uint32_t valid_frames_parsed;
    uint32_t invalid_frames;
    uint32_t engineering_frames_received;

    bool target_confirmed;          // New: Track verified targets
    uint8_t target_confidence;      // New: Confidence percentage (0-100)

} ld2410_sensor_t;

#ifdef __cplusplus
extern "C" {
#endif
// Function declarations
esp_err_t ld2410_init(uint8_t tx_pin, uint8_t rx_pin);
esp_err_t ld2410_init_presence_pin(ld2410_sensor_t *sensor, gpio_num_t presence_pin);
esp_err_t ld2410_init_moving_averages(ld2410_sensor_t *sensor, uint8_t window_size);
void ld2410_cleanup_moving_averages(ld2410_sensor_t *sensor);

esp_err_t ld2410_set_distance_resolution(distance_resolution_t resolution);
esp_err_t ld2410_query_distance_resolution(void);
// Main read function
esp_err_t ld2410_read_data(ld2410_sensor_t *sensor);
bool ld2410_read_presence_pin(ld2410_sensor_t *sensor);

// Frame parsing
void ld2410_readline(ld2410_sensor_t *sensor);
bool ld2410_parse_data_frame(ld2410_sensor_t *sensor, uint8_t *buffer, int len);
bool ld2410_parse_engineering_frame(ld2410_sensor_t *sensor, uint8_t *buffer, int len);

// Configuration functions
esp_err_t ld2410_send_command(uint8_t *command, uint8_t command_length);
esp_err_t ld2410_enable_config_mode(void);
esp_err_t ld2410_disable_config_mode(void);
esp_err_t ld2410_read_parameters(void);
esp_err_t ld2410_enable_engineering_mode(void);
esp_err_t ld2410_disable_engineering_mode(void);
esp_err_t ld2410_set_max_distances(uint8_t max_moving_gate, uint8_t max_stationary_gate);
esp_err_t ld2410_set_gate_sensitivity(uint8_t gate, uint8_t moving_threshold, uint8_t stationary_threshold);
esp_err_t ld2410_set_timeout(uint16_t timeout_seconds);
esp_err_t ld2410_set_light_function(bool enable, uint16_t threshold);
esp_err_t ld2410_set_bluetooth(bool enable);
esp_err_t ld2410_factory_reset(void);
esp_err_t ld2410_restart_sensor(void);

// High-level configuration
esp_err_t ld2410_apply_config_safely(ld2410_config_t *config);
esp_err_t ld2410_load_walking_aid_config(ld2410_config_t *config);

// Moving average functions
esp_err_t moving_average_init(moving_average_t *avg, uint8_t window_size);
void moving_average_cleanup(moving_average_t *avg);
uint16_t moving_average_add_sample(moving_average_t *avg, ld2410_sensor_t *sensor, uint16_t sample);
//uint16_t stabilized_moving_average_add_sample(moving_average_t *avg, uint16_t sample) ;
uint16_t stabilized_moving_average_add_sample(moving_average_t *avg, ld2410_sensor_t *sensor, uint16_t sample) ;
void moving_average_reset(moving_average_t *avg);

#ifdef __cplusplus
}
#endif

#endif // LD2410_H

