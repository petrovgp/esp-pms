/**
 * @file pms.c
 * @author Marko Petrov (markopetrov690@gmail.com)
 * @brief ESP-IDF component for Plantower PMSX003 air quality sensors
 * @version 0.1
 * @date 2025-09-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <pms.h>

#include <string.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_err.h>

// Commands lenght
#define PMS_CMD_LEN             0x07
#define PMS_RESPONSE_LEN        0x08

// Commands
static uint8_t pms_cmd_mode_passive[] = {0x42, 0x4d, 0xe1, 0x00, 0x00, 0x01, 0x70};
static uint8_t pms_cmd_mode_active[]  = {0x42, 0x4d, 0xe1, 0x00, 0x01, 0x01, 0x71};
static uint8_t pms_cmd_state_sleep[]  = {0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73};
static uint8_t pms_cmd_state_active[] = {0x42, 0x4d, 0xe4, 0x00, 0x01, 0x01, 0x74};
static uint8_t pms_cmd_read_passive[] = {0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};

// Data frame lenght in bytes
#define PMS_3003_FRAME_LEN      24
#define PMS_X003_FRAME_LEN      32

// PMS data start bytes
#define PMS_START_BYTE_HIGH     0x42
#define PMS_START_BYTE_LOW      0x4d

// PMS data offsets
#define PMS_PM1_CF1_HIGH_BYTE       0x04    // PM1 concentration CF=1
#define PMS_PM1_CF1_LOW_BYTE        0x05    
#define PMS_PM2_5_CF1_HIGH_BYTE     0x06    // PM2.5 conecetration CF=1
#define PMS_PM2_5_CF1_LOW_BYTE      0x07    
#define PMS_PM10_CF1_HIGH_BYTE      0x08    // PM10 concentration CF=1
#define PMS_PM10_CF1_LOW_BYTE       0x09    
#define PMS_PM1_ATM_HIGH_BYTE       0x0A    // PM1 concentration (atm pressure)
#define PMS_PM1_ATM_LOW_BYTE        0x0B    
#define PMS_PM2_5_ATM_HIGH_BYTE     0x0C    // PM2.5 conecentration (atm pressure)
#define PMS_PM2_5_ATM_LOW_BYTE      0x0D    
#define PMS_PM10_ATM_HIGH_BYTE      0x0E    // PM10 concentration (atm pressure)
#define PMS_PM10_ATM_LOW_BYTE       0x0F    
#define PMS_0_3UM_HIGH_BYTE         0x10    // 0.3um particle concentration in 0.1L air
#define PMS_0_3UM_LOW_BYTE          0x11    
#define PMS_0_5UM_HIGH_BYTE         0x12    // 0.5um particle concnetration in 0.1L air
#define PMS_0_5UM_LOW_BYTE          0x13    
#define PMS_1UM_HIGH_BYTE           0x14    // 1.0um particle concentration in 0.1L air
#define PMS_1UM_LOW_BYTE            0x15    
#define PMS_2_5UM_HIGH_BYTE         0x16    // 2.5um particle concentration in 0.1L air
#define PMS_2_5UM_LOW_BYTE          0x17    
#define PMS_5_0UM_HIGH_BYTE         0x18    // 5.0um particle concentration in 0.1L air
#define PMS_5_0UM_LOW_BYTE          0x19    
#define PMS_10UM_HIGH_BYTE          0x1A    // 10um particle concentration in 0.1L air
#define PMS_10UM_LOW_BYTE           0x1B

#define PMS_TEMP_HIGH_BYTE          0x18    // Temperature (PMS5003T and PMS5003ST)
#define PMS_TEMP_LOW_BYTE           0x19
#define PMS_HUMID_HIGH_BYTE         0x1A    // Humidity (PMS5003T and PMS5003ST)
#define PMS_HUMID_LOW_BYTE          0x1B

// PMS UART baud rate
#define PMS_BAUD_RATE 9600

// PMS reset time (time after which sensor can receive commands, experimental)
#define PMS_RESET_TIME_MS 2000 

// PMS stabilizing time (after reset, before data can be read)
#define PMS_STABILIZE_TIME_MS 30000

static const char *TAG = "pms";

static pms_sensor_t pms_sensor = {0};

static esp_err_t pms_init_control_pin(gpio_num_t pin, uint8_t initial_level){
    if (pin < 0 || pin >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "invalid GPIO number: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    gpio_config_t gpio_conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if(gpio_config(&gpio_conf) != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure GPIO pin %d", pin);
        return ESP_FAIL;
    }
    
    gpio_set_level(pin, initial_level);
    return ESP_OK;
}

static esp_err_t pms_set_control_pin(gpio_num_t pin, uint32_t state){
    return gpio_set_level(pin, state);
}

static uint8_t *pms_get_mode_cmd(pms_mode_e mode){
    if (mode >= PMS_MODE_MAX) {
        ESP_LOGE(TAG, "invalid mode");
        return NULL;
    }

    switch(mode){
        case PMS_MODE_ACTIVE: return pms_cmd_mode_active;
        case PMS_MODE_PASSIVE: return pms_cmd_mode_passive;
        default: break;
    }

    return NULL;
}

static uint8_t *pms_get_state_cmd(pms_state_e state){
    if (state >= PMS_STATE_MAX) {
        ESP_LOGE(TAG, "invalid state");
        return NULL;
    }

    switch(state){
        case PMS_STATE_ACTIVE: return pms_cmd_state_active;
        case PMS_STATE_SLEEP: return pms_cmd_state_sleep;
        default: break;
    }

    return NULL;
}

static esp_err_t pms_send_cmd(uint8_t *command){
    if(uart_write_bytes(pms_sensor.uart_port, (const char *)command, PMS_CMD_LEN) != PMS_CMD_LEN){
        ESP_LOGE(TAG, "failed to send command");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void pms_timer_cb(TimerHandle_t xTimer){
    switch(pms_sensor.state){
        case PMS_STATE_RESETTING:
            ESP_LOGI(TAG, "sensor reset complete, entering stabilizing state");
            xTimerChangePeriod(pms_sensor.reset_timer, PMS_STABILIZE_TIME_MS / portTICK_PERIOD_MS, 0);
            xTimerStart(pms_sensor.reset_timer, 0);
            pms_sensor.state = PMS_STATE_STABILIZING;
            break;

        case PMS_STATE_STABILIZING:
            pms_sensor.state = PMS_STATE_ACTIVE;
            ESP_LOGI(TAG, "sensor stabilization complete, now in active state");
            break;

        default:
            ESP_LOGW(TAG, "reset timer expired in invalid state");
            break;
    }
}

esp_err_t pms_reset(void){
    if (pms_sensor.state == PMS_STATE_SLEEP){
        ESP_LOGW(TAG, "pms sensor is in sleep state, wake up to reset");
        return ESP_ERR_INVALID_STATE;
    }

    if(pms_set_state(PMS_STATE_SLEEP)){
        ESP_LOGE(TAG, "failed to set sensor to sleep state");
        return ESP_FAIL;
    }
    esp_rom_delay_us(10);
    if(pms_set_state(PMS_STATE_ACTIVE)){
        ESP_LOGE(TAG, "failed to set sensor to active state");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pms_set_mode(pms_mode_e mode){
    if (pms_sensor.state == PMS_STATE_SLEEP){
        ESP_LOGW(TAG, "sensor is in sleep state, wake up to change mode");
        return ESP_ERR_INVALID_STATE;
    }
    if (mode >= PMS_MODE_MAX){
        ESP_LOGE(TAG, "invalid mode");
        return ESP_ERR_INVALID_ARG;
    }
    if (pms_sensor.mode == mode)
        return ESP_OK;

    uint8_t *command = pms_get_mode_cmd(mode);
    if (command == NULL){
        ESP_LOGE(TAG, "failed to get mode command");
        return ESP_ERR_INVALID_ARG;
    }
    if(pms_send_cmd(command) != ESP_OK){
        ESP_LOGE(TAG, "failed to send mode command");
        return ESP_ERR_INVALID_RESPONSE;
    }

    pms_sensor.mode = mode;
    return ESP_OK;
}

esp_err_t pms_set_state(pms_state_e state){
    if (pms_sensor.state == state)
        return ESP_OK;

    switch(state){
        case PMS_STATE_RESETTING:
            ESP_LOGW(TAG, "can't set state to resetting, use pms_reset()");
            return ESP_ERR_INVALID_ARG;

        case PMS_STATE_STABILIZING:
            ESP_LOGW(TAG, "stabilizing state is not supported, use active state");
            return ESP_ERR_INVALID_ARG;

        case PMS_STATE_SLEEP:
            if (pms_sensor.state == PMS_STATE_RESETTING){
                ESP_LOGW(TAG, "sensor must not be in resetting state to enter sleep state");
                return ESP_ERR_INVALID_STATE;
            }
            if (pms_sensor.set_gpio != GPIO_NUM_NC){
                pms_set_control_pin(pms_sensor.set_gpio, 0);
            }else{
                uint8_t *command = pms_get_state_cmd(state);
                if (command == NULL){
                    ESP_LOGE(TAG, "failed to get state command");
                    return ESP_ERR_INVALID_ARG;
                }
                if (pms_send_cmd(command) != ESP_OK){
                    ESP_LOGE(TAG, "failed to send command");
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            pms_sensor.state = state;
            return ESP_OK;
            break;

        case PMS_STATE_ACTIVE:
            if (pms_sensor.state != PMS_STATE_SLEEP){
                ESP_LOGW(TAG, "sensor must be in sleep state to enter active state");
                return ESP_ERR_INVALID_STATE;
            }
            if (pms_sensor.set_gpio != GPIO_NUM_NC){
                pms_set_control_pin(pms_sensor.set_gpio, 1);
            }else{
                uint8_t *command = pms_get_state_cmd(state);
                if (command == NULL){
                    ESP_LOGE(TAG, "failed to get state command");
                    return ESP_ERR_INVALID_ARG;
                }
                if (pms_send_cmd(command) != ESP_OK){
                    ESP_LOGE(TAG, "failed to send state command");
                    return ESP_ERR_INVALID_RESPONSE;
                }
            }
            if (xTimerChangePeriod(pms_sensor.reset_timer, PMS_RESET_TIME_MS / portTICK_PERIOD_MS, 0) != pdPASS) {
                ESP_LOGE(TAG, "failed to change reset timer period");
                return ESP_ERR_INVALID_RESPONSE;
            }
            if (xTimerStart(pms_sensor.reset_timer, 0) != pdPASS) {
                ESP_LOGE(TAG, "failed to start reset timer");
                return ESP_ERR_INVALID_RESPONSE;
            }
            pms_sensor.mode = PMS_MODE_ACTIVE;
            pms_sensor.state = PMS_STATE_RESETTING;
            return ESP_OK;
            break;

        default:
            ESP_LOGE(TAG, "invalid state");
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

pms_mode_e pms_get_mode(void){
    return pms_sensor.mode;
}

pms_state_e pms_get_state(void){
    return pms_sensor.state;
}

uart_port_t pms_get_uart_port(void){
    return pms_sensor.uart_port;
}

pms_type_e pms_get_type(void){
    return pms_sensor.type;
}

static uint8_t pms_get_frame_len(void){
    switch(pms_sensor.type){
        case PMS_TYPE_3003: return PMS_3003_FRAME_LEN;
        case PMS_TYPE_5003:
        case PMS_TYPE_5003T: return PMS_X003_FRAME_LEN;
        default: return 0;
    }
}

static esp_err_t pms_verify_checksum(const uint8_t *data, uint8_t len){
    uint16_t checksum = (data[len - 2] << 8) | data[len - 1];
    uint16_t payload = 0;
    
    // Calculate checksum for payload bytes
    for (int i = 0; i < len - 2; i++){
        payload += data[i];
    }

    // Compare checksums
    if(payload != checksum){
        ESP_LOGE(TAG, "invalid checksum for recieved frame");
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

esp_err_t pms_send_passive_read_cmd(void){
    // Check if sensor is in passive mode and active state
    if(pms_sensor.state != PMS_STATE_ACTIVE){
        ESP_LOGE(TAG, "sensor not in active state");
        return ESP_ERR_INVALID_STATE;
    }
    if(pms_sensor.mode != PMS_MODE_PASSIVE){
        ESP_LOGE(TAG, "sensor not in passive mode");
        return ESP_ERR_INVALID_STATE;
    }

    // Send read command
    if(pms_send_cmd(pms_cmd_read_passive) != ESP_OK){
        ESP_LOGE(TAG, "failed to send passive read command");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pms_parse_cmd_response(const uint8_t *data, uint8_t len){
    if(data == NULL){
        ESP_LOGE(TAG, "data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if(len != PMS_RESPONSE_LEN){
        ESP_LOGE(TAG, "invalid command response length: %d", len);
        return ESP_ERR_INVALID_SIZE;
    }

    // Check start bytes
    if(data[0] == PMS_START_BYTE_HIGH && data[1] == PMS_START_BYTE_LOW){
        // Verify checksum
        if(pms_verify_checksum(data, len) != ESP_OK){
            ESP_LOGE(TAG, "received frame failed checksum");
            return ESP_ERR_INVALID_CRC;
        }
    }else{
        ESP_LOGE(TAG, "invalid start bytes: %02x %02x", data[0], data[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

esp_err_t pms_parse_data(const uint8_t *data, uint8_t len){
    if (data == NULL) {
        ESP_LOGE(TAG, "data pointer is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    if (pms_sensor.state != PMS_STATE_ACTIVE) {
        ESP_LOGW(TAG, "sensor not in active state, data may be invalid");
    }
    
    // Check start bytes
    if(data[0] == PMS_START_BYTE_HIGH && data[1] == PMS_START_BYTE_LOW){
        // Check frame length
        uint8_t expected_len = pms_get_frame_len();
        if (len != expected_len) {
            ESP_LOGE(TAG, "expected frame lenght is %u, but got %u", expected_len, len);
            return ESP_ERR_INVALID_SIZE;
        }
        // Verify checksum
        if(pms_verify_checksum(data, len) != ESP_OK){
            ESP_LOGE(TAG, "received frame failed checksum");
            return ESP_ERR_INVALID_CRC;
        }

        // Store raw data frame
        memcpy(pms_sensor.raw_data, data, len * sizeof(uint8_t));
    } else {
        ESP_LOGE(TAG, "invalid start bytes: %02x %02x", data[0], data[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

int16_t pms_get_data(pms_field_t field){
    switch(pms_sensor.type){
        case PMS_TYPE_3003:
            if(field >= PMS_FIELD_PC_0_3){
                ESP_LOGW(TAG, "requested field not available on PMS3003 sensor");
                return 0;
            }
            break;
        case PMS_TYPE_5003:
            if(field >= PMS_FIELD_TEMP){
                ESP_LOGW(TAG, "requested field not available on PMS5003 sensor");
                return 0;
            }
            break;
        case PMS_TYPE_5003T:
            if(field == PMS_FIELD_PC_5_0 || field == PMS_FIELD_PC_10){
                ESP_LOGW(TAG, "requested field not available on PMS5003T sensor");
                return 0;
            }
            break;
        default:
            break;
    }

    switch (field) {
        case PMS_FIELD_PM1_CF1:     return (pms_sensor.raw_data[PMS_PM1_CF1_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_PM1_CF1_LOW_BYTE];
        case PMS_FIELD_PM2_5_CF1:   return (pms_sensor.raw_data[PMS_PM2_5_CF1_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_PM2_5_CF1_LOW_BYTE];
        case PMS_FIELD_PM10_CF1:    return (pms_sensor.raw_data[PMS_PM10_CF1_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_PM10_CF1_LOW_BYTE];

        case PMS_FIELD_PM1_ATM:     return (pms_sensor.raw_data[PMS_PM1_ATM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_PM1_ATM_LOW_BYTE];
        case PMS_FIELD_PM2_5_ATM:   return (pms_sensor.raw_data[PMS_PM2_5_ATM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_PM2_5_ATM_LOW_BYTE];
        case PMS_FIELD_PM10_ATM:    return (pms_sensor.raw_data[PMS_PM10_ATM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_PM10_ATM_LOW_BYTE];

        case PMS_FIELD_PC_0_3:      return (pms_sensor.raw_data[PMS_0_3UM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_0_3UM_LOW_BYTE];
        case PMS_FIELD_PC_0_5:      return (pms_sensor.raw_data[PMS_0_5UM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_0_5UM_LOW_BYTE];
        case PMS_FIELD_PC_1_0:      return (pms_sensor.raw_data[PMS_1UM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_1UM_LOW_BYTE];
        case PMS_FIELD_PC_2_5:      return (pms_sensor.raw_data[PMS_2_5UM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_2_5UM_LOW_BYTE];
        case PMS_FIELD_PC_5_0:      return (pms_sensor.raw_data[PMS_5_0UM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_5_0UM_LOW_BYTE];
        case PMS_FIELD_PC_10:       return (pms_sensor.raw_data[PMS_10UM_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_10UM_LOW_BYTE];

        case PMS_FIELD_TEMP:        return ((pms_sensor.raw_data[PMS_TEMP_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_TEMP_LOW_BYTE]) / 10;
        case PMS_FIELD_HUMIDITY:    return ((pms_sensor.raw_data[PMS_HUMID_HIGH_BYTE] << 8) | pms_sensor.raw_data[PMS_HUMID_LOW_BYTE]) / 10;
        
        default:
            ESP_LOGW(TAG, "invalid data requested");
            return 0;
    }
}

esp_err_t pms_init(pms_config_t *pms_config){
    // Initialize default PMS settings
    pms_sensor.mode = PMS_MODE_ACTIVE;
    pms_sensor.state = PMS_STATE_ACTIVE;
    pms_sensor.type = pms_config->type;
    pms_sensor.uart_port = pms_config->uart_port;
    pms_sensor.set_gpio = pms_config->set_gpio;
    pms_sensor.reset_gpio = pms_config->reset_gpio;

    // Init GPIOs
    if (pms_config->set_gpio != GPIO_NUM_NC){
        if(pms_init_control_pin(pms_config->set_gpio, 1) != ESP_OK){
            ESP_LOGE(TAG, "failed to init control pin %d", pms_config->set_gpio);
            return ESP_ERR_INVALID_ARG;
        }
    }

    if(pms_config->reset_gpio != GPIO_NUM_NC){
        if(pms_init_control_pin(pms_config->reset_gpio, 1) != ESP_OK){
            ESP_LOGE(TAG, "failed to init control pin %d", pms_config->reset_gpio);
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Init timer for wake up delay
    pms_sensor.reset_timer = xTimerCreate(
        "pms_timer",
        PMS_RESET_TIME_MS / portTICK_PERIOD_MS,
        pdFALSE,
        NULL,
        pms_timer_cb
    );
    if (pms_sensor.reset_timer == NULL) {
        ESP_LOGE(TAG, "failed to create reset timer");
        return ESP_ERR_NO_MEM;
    }

    // Reset PMS sensor
    if(pms_reset() != ESP_OK){
        ESP_LOGE(TAG, "failed to reset sensor");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t pms_deinit(void){
    // Deinit GPIOs
    if (pms_sensor.set_gpio != GPIO_NUM_NC){
        gpio_reset_pin(pms_sensor.set_gpio);
    }
    if (pms_sensor.reset_gpio != GPIO_NUM_NC){
        gpio_reset_pin(pms_sensor.reset_gpio);
    }

    // Delete timer
    if (pms_sensor.reset_timer != NULL){
        xTimerStop(pms_sensor.reset_timer, 0);
        xTimerDelete(pms_sensor.reset_timer, 0);
        pms_sensor.reset_timer = NULL;
    }
    
    // Reset struct
    pms_sensor = (pms_sensor_t){
        .set_gpio = GPIO_NUM_NC,
        .reset_gpio = GPIO_NUM_NC,
        .uart_port = UART_NUM_MAX,
        .type = PMS_TYPE_MAX,
        .state = PMS_STATE_MAX,
        .mode = PMS_MODE_MAX,
        .raw_data = {0}
    };

    return ESP_OK;
}