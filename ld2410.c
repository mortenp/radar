#include "ld2410.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "LD2410";

//esp_log_level_set(TAG, ESP_LOG_DEBUG);


// Moving average functions (same as before)
esp_err_t moving_average_init(moving_average_t *avg, uint8_t window_size) {
    if (avg == NULL || window_size == 0 || window_size > MAX_WINDOW_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }
    
    avg->buffer = (uint16_t*)malloc(window_size * sizeof(uint16_t));
    if (avg->buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for moving average buffer");
        return ESP_ERR_NO_MEM;
    }
    
    memset(avg->buffer, 0, window_size * sizeof(uint16_t));
    avg->window_size = window_size;
    avg->current_index = 0;
    avg->count = 0;
    avg->sum = 0;
    avg->is_full = false;
    avg->average = 0;
    avg->last_value = 0;
    avg->trend = 0;
    
    return ESP_OK;
}

void moving_average_cleanup(moving_average_t *avg) {
    if (avg != NULL && avg->buffer != NULL) {
        free(avg->buffer);
        avg->buffer = NULL;
    }
}

uint16_t moving_average_add_sample(moving_average_t *avg, uint16_t sample) {
    if (avg == NULL || avg->buffer == NULL) {
        return sample;
    }
    
    if (avg->count > 0) {
        avg->trend = (int16_t)sample - (int16_t)avg->last_value;
    }
    avg->last_value = sample;
    
    if (avg->is_full) {
        avg->sum -= avg->buffer[avg->current_index];
    }
    
    avg->buffer[avg->current_index] = sample;
    avg->sum += sample;
    
    avg->current_index = (avg->current_index + 1) % avg->window_size;
    
    if (!avg->is_full) {
        avg->count++;
        if (avg->count >= avg->window_size) {
            avg->is_full = true;
        }
    }
    
    uint8_t divisor = avg->is_full ? avg->window_size : avg->count;
    avg->average = (divisor > 0) ? (avg->sum / divisor) : sample;
    
    return avg->average;
}

void moving_average_reset(moving_average_t *avg) {
    if (avg != NULL && avg->buffer != NULL) {
        memset(avg->buffer, 0, avg->window_size * sizeof(uint16_t));
        avg->current_index = 0;
        avg->count = 0;
        avg->sum = 0;
        avg->is_full = false;
        avg->average = 0;
        avg->last_value = 0;
        avg->trend = 0;
    }
}

////////   settings ///////
// Enhanced command sending with response waiting
esp_err_t ld2410_send_command_with_response(uint8_t *command, uint8_t command_length, uint32_t timeout_ms) {
    // Send command
    int written = uart_write_bytes(LD2410_UART_NUM, command, command_length);
    if (written != command_length) {
        ESP_LOGE(TAG, "Failed to send command, wrote %d/%d bytes", written, command_length);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Sent command:");
    ESP_LOG_BUFFER_HEX(TAG, command, command_length);
    
    // Wait for response
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    
    // Read and display response
    uint8_t response[64];
    int bytes_read = uart_read_bytes(LD2410_UART_NUM, response, sizeof(response), 500 / portTICK_PERIOD_MS);
    if (bytes_read > 0) {
        ESP_LOGI(TAG, "Received response (%d bytes):", bytes_read);
        ESP_LOG_BUFFER_HEX(TAG, response, bytes_read);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "No response received");
        return ESP_ERR_TIMEOUT;
    }
}

// Configuration command functions
esp_err_t ld2410_set_max_distances(uint8_t max_moving_gate, uint8_t max_stationary_gate) {
    uint8_t cmd[] = {
        0xFA, 0xFB, 0xFC, 0xFD,  // Header
        0x14, 0x00,              // Length (20 bytes)
        0x60, 0x00,              // Command: set max distances
        0x00, 0x00,              // Reserved
        max_moving_gate, 0x00, 0x00, 0x00,     // Max moving gate
        0x01, 0x00,              // Parameter 1
        max_stationary_gate, 0x00, 0x00, 0x00, // Max stationary gate  
        0x02, 0x00,              // Parameter 2
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGI(TAG, "Setting max distances: moving=%d, stationary=%d", max_moving_gate, max_stationary_gate);
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_set_distance_resolution(distance_resolution_t resolution) {
    uint8_t cmd[] = {
        0xFA, 0xFB, 0xFC, 0xFD,  // Header
        0x08, 0x00,              // Length (8 bytes)
        0xAA, 0x00,              // Command: set distance resolution
        0x00, 0x00,              // Reserved
        resolution, 0x00, 0x00, 0x00,  // Resolution value
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    const char* res_str = (resolution == DISTANCE_RESOLUTION_0_2) ? "0.2m" : "0.75m";
    ESP_LOGI(TAG, "Setting distance resolution: %s", res_str);
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_query_distance_resolution(void) {
    uint8_t cmd[] = {
        0xFA, 0xFB, 0xFC, 0xFD,  // Header
        0x02, 0x00,              // Length (2 bytes)
        0xAB, 0x00,              // Command: query distance resolution
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGI(TAG, "Querying distance resolution");
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_set_gate_sensitivity(uint8_t gate, uint8_t moving_threshold, uint8_t stationary_threshold) {
    if (gate >= LD2410_MAX_GATES) {
        ESP_LOGE(TAG, "Invalid gate number: %d (max: %d)", gate, LD2410_MAX_GATES - 1);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t cmd[] = {
        0xFA, 0xFB, 0xFC, 0xFD,  // Header
        0x14, 0x00,              // Length (20 bytes)
        0x64, 0x00,              // Command: set gate sensitivity
        0x00, 0x00,              // Reserved
        gate, 0x00, 0x00, 0x00,  // Gate number
        0x01, 0x00,              // Parameter 1
        moving_threshold, 0x00, 0x00, 0x00,     // Moving threshold
        0x02, 0x00,              // Parameter 2
        stationary_threshold, 0x00, 0x00, 0x00, // Stationary threshold
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGI(TAG, "Setting gate %d sensitivity: moving=%d, stationary=%d", gate, moving_threshold, stationary_threshold);
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_enable_engineering_mode(void) {
    uint8_t cmd[] = {0xFA, 0xFB, 0xFC, 0xFD, 0x02, 0x00, 0x62, 0x00, 0x04, 0x03, 0x02, 0x01};
    ESP_LOGI(TAG, "Enabling engineering mode");
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_disable_engineering_mode(void) {
    uint8_t cmd[] = {0xFA, 0xFB, 0xFC, 0xFD, 0x02, 0x00, 0x63, 0x00, 0x04, 0x03, 0x02, 0x01};
    ESP_LOGI(TAG, "Disabling engineering mode");
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_set_light_function(bool enable, uint16_t threshold) {
    uint8_t cmd[] = {
        0xFA, 0xFB, 0xFC, 0xFD,  // Header
        0x14, 0x00,              // Length
        0xA5, 0x00,              // Command: set light function
        0x00, 0x00,              // Reserved
        enable ? 0x01 : 0x00, 0x00, 0x00, 0x00,  // Enable/disable
        0x01, 0x00,              // Parameter 1
        threshold & 0xFF, (threshold >> 8) & 0xFF, 0x00, 0x00,  // Threshold
        0x02, 0x00,              // Parameter 2
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGI(TAG, "Setting light function: %s, threshold=%d", enable ? "enabled" : "disabled", threshold);
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_set_bluetooth(bool enable) {
    uint8_t cmd[] = {
        0xFA, 0xFB, 0xFC, 0xFD,  // Header
        0x08, 0x00,              // Length
        0xA4, 0x00,              // Command: set bluetooth
        0x00, 0x00,              // Reserved
        enable ? 0x01 : 0x00, 0x00, 0x00, 0x00,  // Enable/disable
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGI(TAG, "Setting bluetooth: %s", enable ? "enabled" : "disabled");
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 200);
}

esp_err_t ld2410_factory_reset(void) {
    uint8_t cmd[] = {0xFA, 0xFB, 0xFC, 0xFD, 0x02, 0x00, 0xA2, 0x00, 0x04, 0x03, 0x02, 0x01};
    ESP_LOGI(TAG, "Performing factory reset");
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 1000);
}

esp_err_t ld2410_restart_sensor(void) {
    uint8_t cmd[] = {0xFA, 0xFB, 0xFC, 0xFD, 0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};
    ESP_LOGI(TAG, "Restarting sensor");
    return ld2410_send_command_with_response(cmd, sizeof(cmd), 1000);
}

// Load optimized walking aid configuration
esp_err_t ld2410_load_walking_aid_config(ld2410_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    // Default walking aid configuration
    config->max_move_distance_gate = 8;      // Use all gates for movement
    config->max_still_distance_gate = 8;     // Shorter range for stationary objects
    config->timeout = 5;                     // 5 second timeout
    config->light_function_enabled = false;  // Disable light compensation initially
    config->light_threshold = 128;           // Default light threshold
    config->bluetooth_enabled = false;       // Disable bluetooth for power saving
    config->engineering_mode = false;        // Start in normal mode
    config->distance_resolution = DISTANCE_RESOLUTION_0_2;  // High precision for walking aid
    
    // Sensitivity configuration optimized for walking aid
    for (int gate = 0; gate < LD2410_MAX_GATES; gate++) {
        if (gate <= 2) {
            // Close range: high sensitivity for safety
            config->move_thresholds[gate] = 50;
            config->still_thresholds[gate] = 0;
        } else if (gate <= 5) {
            // Medium range: moderate sensitivity
            config->move_thresholds[gate] = 40;
            config->still_thresholds[gate] = 40;
        } else {
            // Far range: lower sensitivity to reduce false alarms
            config->move_thresholds[gate] = 50;
            config->still_thresholds[gate] = 35;
        }
    }
    
    ESP_LOGI(TAG, "Loaded walking aid configuration");
    return ESP_OK;
}

// Apply complete configuration to sensor
esp_err_t ld2410_apply_config_safely(ld2410_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    ESP_LOGI(TAG, "🔧 Safely applying LD2410 configuration...");
    
    // Test current communication first
    uint8_t test_buffer[64];
    int test_length = uart_read_bytes(LD2410_UART_NUM, test_buffer, sizeof(test_buffer), 200 / portTICK_PERIOD_MS);
    if (test_length <= 0) {
        ESP_LOGE(TAG, "❌ No communication before config - aborting");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "✅ Communication OK before config");
    
    // Enable configuration mode
    esp_err_t ret = ld2410_enable_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable config mode");
        return ret;
    }
    ESP_LOGI(TAG, "✅ Config mode enabled");
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // Apply configuration step by step with verification
    
    // 1. Set distance resolution (do this first)
    ESP_LOGI(TAG, "Setting distance resolution...");
    ret = ld2410_set_distance_resolution(config->distance_resolution);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Failed to set distance resolution, continuing...");
    } else {
        ESP_LOGI(TAG, "✅ Distance resolution set to %s", 
                 (config->distance_resolution == DISTANCE_RESOLUTION_0_2) ? "0.2m" : "0.75m");
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // 2. Set max distances
    ESP_LOGI(TAG, "Setting max distances...");
    ret = ld2410_set_max_distances(config->max_move_distance_gate, config->max_still_distance_gate);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "⚠️ Failed to set max distances, continuing...");
    } else {
        ESP_LOGI(TAG, "✅ Max distances set");
    }
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // 3. Set gate sensitivities for close range only (gates 0-2)
    ESP_LOGI(TAG, "Setting close-range gate sensitivities...");
    for (int gate = 0; gate < 3; gate++) {
        ret = ld2410_set_gate_sensitivity(gate, config->move_thresholds[gate], config->still_thresholds[gate]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "⚠️ Failed to set gate %d sensitivity", gate);
        } else {
            ESP_LOGD(TAG, "✅ Gate %d sensitivity set", gate);
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // 4. Disable configuration mode
    ESP_LOGI(TAG, "Disabling config mode...");
    ret = ld2410_disable_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable config mode");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 5. Test communication after config
    ESP_LOGI(TAG, "Testing communication after config...");
    uint8_t verify_buffer[64];
    int verify_length = uart_read_bytes(LD2410_UART_NUM, verify_buffer, sizeof(verify_buffer), 1000 / portTICK_PERIOD_MS);
    if (verify_length > 0) {
        // Look for valid frames
        for (int i = 0; i <= verify_length - 4; i++) {
            if (verify_buffer[i] == 0xF4 && verify_buffer[i+1] == 0xF3 && 
                verify_buffer[i+2] == 0xF2 && verify_buffer[i+3] == 0xF1) {
                ESP_LOGI(TAG, "✅ Configuration successful - sensor responding normally");
                return ESP_OK;
            }
        }
        ESP_LOGW(TAG, "⚠️ Sensor responding but frame format may have changed");
        ESP_LOG_BUFFER_HEX(TAG, verify_buffer, verify_length > 16 ? 16 : verify_length);
    } else {
        ESP_LOGE(TAG, "❌ No response after configuration");
    }
    
    return ESP_OK;  // Continue anyway
}

// Parse engineering mode data
bool ld2410_parse_engineering_frame(ld2410_sensor_t *sensor, uint8_t *buffer, int len) {
    if (len < 23) {  // Engineering frame should be at least 23 bytes
        ESP_LOGW(TAG, "Engineering frame too short: %d bytes", len);
        return false;
    }
    
    // Skip first byte (0xAA) like regular frames
    uint8_t *data = &buffer[1];
    
    sensor->engineering_data.max_moving_gate = data[0];
    sensor->engineering_data.max_stationary_gate = data[1];
    
    // Parse gate energies
    for (int gate = 0; gate < LD2410_MAX_GATES; gate++) {
        if (2 + gate * 2 + 1 < len - 1) {
            sensor->engineering_data.moving_energy[gate] = data[2 + gate * 2];
            sensor->engineering_data.stationary_energy[gate] = data[2 + gate * 2 + 1];
        }
    }
    
    // Parse additional engineering data if available
    int light_pos = 2 + LD2410_MAX_GATES * 2;
    if (light_pos + 1 < len - 1) {
        sensor->engineering_data.light_value = data[light_pos] | (data[light_pos + 1] << 8);
    }
    
    if (light_pos + 2 < len - 1) {
        sensor->engineering_data.out_pin_level = data[light_pos + 2];
    }
    
    sensor->engineering_frames_received++;
    
    ESP_LOGD(TAG, "📊 Engineering data: MaxMove=%d, MaxStill=%d, Light=%d",
             sensor->engineering_data.max_moving_gate,
             sensor->engineering_data.max_stationary_gate,
             sensor->engineering_data.light_value);
    
    return true;
}


esp_err_t ld2410_init_presence_pin(ld2410_sensor_t *sensor, gpio_num_t presence_pin) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << presence_pin),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure presence detection pin GPIO%d", presence_pin);
        return ret;
    }
    
    sensor->presence_pin = presence_pin;
    sensor->presence_detected = false;
    sensor->presence_pin_enabled = true;
    sensor->presence_high_count = 0;
    sensor->presence_low_count = 0;
    sensor->presence_total_reads = 0;
    
    ESP_LOGI(TAG, "Presence detection pin initialized on GPIO%d", presence_pin);
    return ESP_OK;
}

bool ld2410_read_presence_pin(ld2410_sensor_t *sensor) {
    if (sensor == NULL || !sensor->presence_pin_enabled) {
        return false;
    }
    
    int pin_level = gpio_get_level(sensor->presence_pin);
    sensor->presence_detected = (pin_level == 1);
    
    sensor->presence_total_reads++;
    if (sensor->presence_detected) {
        sensor->presence_high_count++;
    } else {
        sensor->presence_low_count++;
    }
    
    return sensor->presence_detected;
}

esp_err_t ld2410_init_moving_averages(ld2410_sensor_t *sensor, uint8_t window_size) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    ret = moving_average_init(&sensor->moving_distance_avg, window_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize moving distance average");
        return ret;
    }
    
    ret = moving_average_init(&sensor->stationary_distance_avg, window_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize stationary distance average");
        moving_average_cleanup(&sensor->moving_distance_avg);
        return ret;
    }
    
    ret = moving_average_init(&sensor->detection_distance_avg, window_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize detection distance average");
        moving_average_cleanup(&sensor->moving_distance_avg);
        moving_average_cleanup(&sensor->stationary_distance_avg);
        return ret;
    }
    
    ESP_LOGI(TAG, "Moving averages initialized with window size: %d", window_size);
    return ESP_OK;
}

void ld2410_cleanup_moving_averages(ld2410_sensor_t *sensor) {
    if (sensor != NULL) {
        moving_average_cleanup(&sensor->moving_distance_avg);
        moving_average_cleanup(&sensor->stationary_distance_avg);
        moving_average_cleanup(&sensor->detection_distance_avg);
    }
}

esp_err_t ld2410_init(uint8_t tx_pin, uint8_t rx_pin) {
    ESP_LOGI(TAG, "Initializing LD2410 UART...");
    ESP_LOGI(TAG, "TX Pin: GPIO%d, RX Pin: GPIO%d", tx_pin, rx_pin);
    
    if (uart_is_driver_installed(LD2410_UART_NUM)) {
        ESP_LOGW(TAG, "UART driver already installed, uninstalling first...");
        uart_driver_delete(LD2410_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    uart_config_t uart_config = {
        .baud_rate = LD2410_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(LD2410_UART_NUM, LD2410_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(LD2410_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        uart_driver_delete(LD2410_UART_NUM);
        return ret;
    }

    ret = uart_set_pin(LD2410_UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(LD2410_UART_NUM);
        return ret;
    }

    uart_flush(LD2410_UART_NUM);
    ESP_LOGI(TAG, "LD2410 UART initialized successfully");
    return ESP_OK;
}

bool ld2410_parse_data_frame(ld2410_sensor_t *sensor, uint8_t *buffer, int len) {
    // Both 12 and 13-byte frames need to skip the first byte (0xAA)
    if (len < 12) {
        ESP_LOGW(TAG, "Frame too short: %d bytes (need at least 12)", len);
        return false;
    }
    
    // Always skip first byte for both 12 and 13-byte frames
    uint8_t *data = &buffer[1];
    int data_len = len - 1; // Available data after skipping first byte
    
    if (data_len < 11) {
        ESP_LOGW(TAG, "Not enough data after skipping first byte: %d bytes (need 11)", data_len);
        return false;
    }
    
    ESP_LOGD(TAG, "Parsing %d-byte frame (skipping first byte: 0x%02X)", len, buffer[0]);
    
    // Parse sensor data (always skip first byte method)
    sensor->target_data.target_state = (ld2410_target_state_t)data[0];
    sensor->target_data.raw_moving_target_distance = data[1] | (data[2] << 8);
    sensor->target_data.moving_target_energy = data[3];
    sensor->target_data.raw_stationary_target_distance = data[4] | (data[5] << 8);
    sensor->target_data.stationary_target_energy = data[6];
    sensor->target_data.raw_detection_distance = data[7] | (data[8] << 8);
    
    // Validate the parsed data
    if (sensor->target_data.target_state > 3 || 
        sensor->target_data.raw_moving_target_distance > 2000 ||
        sensor->target_data.raw_stationary_target_distance > 2000 ||
        sensor->target_data.raw_detection_distance > 2000 ||
        sensor->target_data.moving_target_energy > 100 ||
        sensor->target_data.stationary_target_energy > 100) {
        
        ESP_LOGW(TAG, "Invalid data (%d bytes): State:%d, Move:%dcm/%d, Still:%dcm/%d, Det:%dcm", 
                 len,
                 sensor->target_data.target_state,
                 sensor->target_data.raw_moving_target_distance, sensor->target_data.moving_target_energy,
                 sensor->target_data.raw_stationary_target_distance, sensor->target_data.stationary_target_energy,
                 sensor->target_data.raw_detection_distance);
        
        ESP_LOGI(TAG, "Raw frame data for debugging:");
        ESP_LOG_BUFFER_HEX(TAG, buffer, len > 16 ? 16 : len);
        
        return false;
    }
    
    // Apply moving averages

    ESP_LOGD(TAG, "Apply average: moving_distance_avg: %d ): moving_target_distance: %d, raw_moving_target_distance: %d raw_stationary_target_distance: %d, detection_distance: %d, detection_distance_avg: %d", 
             len,
             sensor->moving_distance_avg,
             sensor->target_data.moving_target_distance, 
             sensor->target_data.raw_moving_target_distance,
             sensor->target_data.raw_stationary_target_distance, 
          
             sensor->target_data.detection_distance,
            sensor->detection_distance_avg
);


    sensor->target_data.moving_target_distance = moving_average_add_sample(
        &sensor->moving_distance_avg, sensor->target_data.raw_moving_target_distance);

    sensor->target_data.stationary_target_distance = moving_average_add_sample(
        &sensor->stationary_distance_avg, sensor->target_data.raw_stationary_target_distance);

    sensor->target_data.detection_distance = moving_average_add_sample(
        &sensor->detection_distance_avg, sensor->target_data.raw_detection_distance);
    
    ESP_LOGD(TAG, "✅ PARSED (%d bytes): State=%d, Move=%dcm/%d, Still=%dcm/%d, Det=%dcm", 
             len,
             sensor->target_data.target_state,
             sensor->target_data.moving_target_distance, sensor->target_data.moving_target_energy,
             sensor->target_data.stationary_target_distance, sensor->target_data.stationary_target_energy,
             sensor->target_data.detection_distance);
    
    return true;
}


void debug_frame_variations(uint8_t *buffer, int len) {
    static int frame_count = 0;
    static int last_length = 0;
    
    frame_count++;
    
    if (len != last_length) {
        ESP_LOGI(TAG, "📏 Frame length changed: %d -> %d bytes (frame #%d)", 
                 last_length, len, frame_count);
        ESP_LOGI(TAG, "Raw data:");
        ESP_LOG_BUFFER_HEX(TAG, buffer, len > 16 ? 16 : len);
        
        ESP_LOGI(TAG, "First byte: 0x%02X (%d)", buffer[0], buffer[0]);
        
        if (len >= 12) {
            ESP_LOGI(TAG, "Skip-first method: State=%d, Move=%dcm/%d, Still=%dcm/%d, Det=%dcm",
                     buffer[1],
                     buffer[2] | (buffer[3] << 8), buffer[4],
                     buffer[5] | (buffer[6] << 8), buffer[7],
                     buffer[8] | (buffer[9] << 8));
        }
        
        last_length = len;
    }
}


// ESPHome-style readline function
void ld2410_readline(ld2410_sensor_t *sensor) {
    uint8_t buffer[64]; // overflow?
    int available = uart_read_bytes(LD2410_UART_NUM, buffer, sizeof(buffer), 10 / portTICK_PERIOD_MS);
    
    if (available <= 0) {
        return;
    }
    
    for (int i = 0; i < available; i++) {
        uint8_t byte = buffer[i];
        
        if (sensor->rx_buffer_position < sizeof(sensor->rx_buffer)) {
            sensor->rx_buffer[sensor->rx_buffer_position++] = byte;
        } else {
            // Buffer overflow, reset
            ESP_LOGW(TAG, "RX buffer overflow, resetting");
            sensor->rx_buffer_position = 0;
            sensor->invalid_frames++;
            continue;
        }
        
        // Check for complete frame
        if (sensor->rx_buffer_position >= 4) {
            // Check for frame header (F4 F3 F2 F1)
            if (sensor->rx_buffer[0] == 0xF4 && sensor->rx_buffer[1] == 0xF3 && 
                sensor->rx_buffer[2] == 0xF2 && sensor->rx_buffer[3] == 0xF1) {
                
                // We have a header, check if we have length bytes
                if (sensor->rx_buffer_position >= 6) {
                    uint16_t frame_length = sensor->rx_buffer[4] | (sensor->rx_buffer[5] << 8);
                    uint16_t total_frame_length = frame_length + 10; // header(4) + length(2) + data + footer(4)
                    
                    if (sensor->rx_buffer_position >= total_frame_length) {
                        // Check footer (F8 F7 F6 F5)
                        uint16_t footer_pos = total_frame_length - 4;
                        if (sensor->rx_buffer[footer_pos] == 0xF8 && 
                            sensor->rx_buffer[footer_pos + 1] == 0xF7 &&
                            sensor->rx_buffer[footer_pos + 2] == 0xF6 && 
                            sensor->rx_buffer[footer_pos + 3] == 0xF5) {
                            
                            // Valid frame found
                            sensor->total_frames_received++;
                            
                            uint8_t frame_type = sensor->rx_buffer[6];
                            ESP_LOGD(TAG, "Valid frame: type=0x%02X, length=%d", frame_type, frame_length);
                            
                            // Parse both data frames (0x01) AND command frames (0x02) as sensor data
                            // Since Method 2 works with command frames
                            if (frame_type == LD2410_DATA_FRAME || frame_type == LD2410_CMD_FRAME) {

    debug_frame_variations(&sensor->rx_buffer[7], frame_length - 1);


                                if (ld2410_parse_data_frame(sensor, &sensor->rx_buffer[7], frame_length - 1)) {
                                    sensor->valid_frames_parsed++;
                                    sensor->is_connected = true;
                                    ESP_LOGD(TAG, "Frame parsed successfully (type: 0x%02X)", frame_type);
                                } else {
                                    sensor->invalid_frames++;
                                    ESP_LOGW(TAG, "Failed to parse frame (type: 0x%02X)", frame_type);
                                }
                            } else {
                                ESP_LOGD(TAG, "Unknown frame type: 0x%02X", frame_type);
                            }
                            
                            // Remove processed frame from buffer
                            int remaining = sensor->rx_buffer_position - total_frame_length;
                            if (remaining > 0) {
                                memmove(sensor->rx_buffer, &sensor->rx_buffer[total_frame_length], remaining);
                            }
                            sensor->rx_buffer_position = remaining;
                            
                        } else {
                            // Invalid footer, discard one byte and continue
                            ESP_LOGD(TAG, "Invalid footer at pos %d", footer_pos);
                            memmove(sensor->rx_buffer, &sensor->rx_buffer[1], sensor->rx_buffer_position - 1);
                            sensor->rx_buffer_position--;
                            sensor->invalid_frames++;
                        }
                    }
                }
            } else {
                // No header found at start of buffer, shift buffer by one byte
                memmove(sensor->rx_buffer, &sensor->rx_buffer[1], sensor->rx_buffer_position - 1);
                sensor->rx_buffer_position--;
            }
        }
    }
}

esp_err_t ld2410_read_data(ld2410_sensor_t *sensor) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Always read presence pin
    if (sensor->presence_pin_enabled) {
        ld2410_read_presence_pin(sensor);
    }
    
    // Read and parse UART data using ESPHome approach
    ld2410_readline(sensor);
    
    // Check if we're getting data
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (current_time - sensor->last_periodic_millis > 1000) {
        if (sensor->total_frames_received == 0) {
            sensor->is_connected = false;
            return ESP_ERR_TIMEOUT;
        }
        sensor->last_periodic_millis = current_time;
    }
    
    return sensor->valid_frames_parsed > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t ld2410_send_command(uint8_t *command, uint8_t command_length) {
    int written = uart_write_bytes(LD2410_UART_NUM, command, command_length);
    if (written != command_length) {
        ESP_LOGE(TAG, "Failed to send command, wrote %d/%d bytes", written, command_length);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t ld2410_enable_config_mode(void) {
    uint8_t cmd[] = {0xFA, 0xFB, 0xFC, 0xFD, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    return ld2410_send_command(cmd, sizeof(cmd));
}

esp_err_t ld2410_disable_config_mode(void) {
    uint8_t cmd[] = {0xFA, 0xFB, 0xFC, 0xFD, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    return ld2410_send_command(cmd, sizeof(cmd));
}