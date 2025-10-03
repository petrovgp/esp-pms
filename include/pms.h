/**
 * @file pms.h
 * @author Marko Petrov (markopetrov690@gmail.com)
 * @brief ESP-IDF component for Plantower PMSX003 air quality sensors
 * @version 1.0
 * @date 2025-09-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef PMS_H_
#define PMS_H_

/**
 * @brief PMSX003 Sensor Short Description
 * 
 * PMSX003 sensors measures concentraions of PM1, PM2.5 and PM10 particles in the air and sends data on an 
 * UART line with TTL levels of 3.3V, in two possible modes: active and passive.
 * 
 * In active mode, sensor measures and sends data over UART continuously. In this mode, it can run in two 
 * sub-modes: stable and fast.
 *  Stable mode  -> small changes in consecutive measurements with data being sent over UART every ~2.3s
 *  Fast mode    -> big changes in consecutive measurements with data being sent over UART every 200~800ms
 * 
 * In passive mode, sensor measures and sens data over UART only when receiving a passive read command. 
 * 
 * A reference schematic, as well as other sensor information, can be found in the documentation for the sensors.
 * 
 * @note 10k pull resistors are recommneded on GPIO lines SET and RESET for better system stability
 */

/**
 * @brief Supported sensors
 * 
 * PMS3003 - supports only PMS3003 sensor with 24-bit data frame
 * PMS5003 - supports all PMSX003 sensors with 32-bit data frame (not all data might be available from each variant)
 */

/**
 * @brief PMSX003 Sensor Data Frame (except PMS3003)
 *
 * Each frame is 32 bytes long and has the following structure:
 *
 * @verbatim
 * +---------+------------+---------------------------------------------------------------+
 * | Byte(s) | Field      | Description                                                   |
 * +---------+------------+---------------------------------------------------------------+
 * | 0       | Start 1    | Start character (0x42)                                        |
 * | 1       | Start 2    | Start character (0x4D)                                        |
 * | 2-3     | Length     | Frame length (always 28)                                      |
 * | 4-5     | Data1      | PM1.0 concentration (CF=1, standard particles, µg/m³)         |
 * | 6-7     | Data2      | PM2.5 concentration (CF=1, standard particles, µg/m³)         |
 * | 8-9     | Data3      | PM10 concentration (CF=1, standard particles, µg/m³)          |
 * | 10-11   | Data4      | PM1.0 concentration (atmospheric environment, µg/m³)          |
 * | 12-13   | Data5      | PM2.5 concentration (atmospheric environment, µg/m³)          |
 * | 14-15   | Data6      | PM10 concentration (atmospheric environment, µg/m³)           |
 * | 16-17   | Data7      | Number of particles >0.3 µm in 0.1 L air                      |
 * | 18-19   | Data8      | Number of particles >0.5 µm in 0.1 L air                      |
 * | 20-21   | Data9      | Number of particles >1.0 µm in 0.1 L air                      |
 * | 22-23   | Data10     | Number of particles >2.5 µm in 0.1 L air                      |
 * | 24-25   | Data11     | Number of particles >5.0 µm in 0.1 L air                      |
 * | 26-27   | Data12     | Number of particles >10 µm in 0.1 L air                       |
 * | 28-29   | Reserved   | Reserved                                                      |
 * | 30-31   | Checksum   | Sum of bytes [0] through [29]                                 |
 * +---------+------------+---------------------------------------------------------------+
 * @endverbatim
 * 
 * @note Total frame size = 32 bytes
 */

/**
 * @brief PMS3003 Sensor Data Frame (Active Mode)
 *
 * Each frame is 24 bytes long and has the following structure:
 *
 * @verbatim
 * +---------+----------------+-------------------------------------------------------------+
 * | Byte(s) | Field          | Description                                                 |
 * +---------+----------------+-------------------------------------------------------------+
 * | 0       | Start 1        | Start character 1 (0x42)                                    |
 * | 1       | Start 2        | Start character 2 (0x4D)                                    |
 * | 2-3     | Length         | Frame length (always 20)                                    |
 * | 4-5     | Data1          | PM1.0 concentration (CF=1, standard particles, µg/m³)       |
 * | 6-7     | Data2          | PM2.5 concentration (CF=1, standard particles, µg/m³)       |
 * | 8-9     | Data3          | PM10 concentration (CF=1, standard particles, µg/m³)        |
 * | 10-11   | Data4          | PM1.0 concentration (atmospheric environment, µg/m³)        |
 * | 12-13   | Data5          | PM2.5 concentration (atmospheric environment, µg/m³)        |
 * | 14-15   | Data6          | PM10 concentration (atmospheric environment, µg/m³)         |
 * | 16-17   | Reserved       | Reserved                                                    |
 * | 18-19   | Reserved       | Reserved                                                    |
 * | 20-21   | Reserved       | Reserved                                                    |
 * | 22-23   | Checksum       | Sum of bytes [0] through [21]                               |
 * +---------+----------------+-------------------------------------------------------------+
 * @endverbatim
 *
 * @note Total frame size = 24 bytes
 */

#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/timers.h>
#include <esp_err.h>
#include <esp_log.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief PMS sensor type
 */
typedef enum{
    PMS_TYPE_1003 = 0,      // PMS1003 type sensor
    PMS_TYPE_3003,          // PMS3003 type sensor
    PMS_TYPE_5003,          // PMS5003 type sensor
    PMS_TYPE_5003T,         // PMS5003T type sensor
    PMS_TYPE_6003,          // PMS6003 type sensor
    PMS_TYPE_7003,          // PMS7003 type sensor
    PMS_TYPE_9003M,         // PMS9003M type sensor
    PMS_TYPE_A003,          // PMSA003 type sensor
    PMS_TYPE_MAX
} pms_type_e;

/**
 * @brief PMS working mode
 */
typedef enum{
    PMS_MODE_PASSIVE = 0,   // Passive working mode
    PMS_MODE_ACTIVE,        // Active working mode
    PMS_MODE_MAX
} pms_mode_e;

/**
 * @brief PMS state 
 */
typedef enum{
    PMS_STATE_SLEEP = 0,    // Sleep state
    PMS_STATE_RESETTING,    // Resetting state
    PMS_STATE_STABILIZING,  // Stabilizing state
    PMS_STATE_ACTIVE,       // Active state
    PMS_STATE_MAX
} pms_state_e;

/**
 * @brief PMS field ID
 */
typedef enum {
    PMS_FIELD_PM1_CF1 = 0,  // PM1.0 concentration (CF=1, standard particles, µg/m³)
    PMS_FIELD_PM2_5_CF1,    // PM2.5 concentration (CF=1, standard particles, µg/m³)
    PMS_FIELD_PM10_CF1,     // PM10 concentration (CF=1, standard particles, µg/m³)
    PMS_FIELD_PM1_ATM,      // PM1.0 concentration (atmospheric environment, µg/m³)
    PMS_FIELD_PM2_5_ATM,    // PM2.5 concentration (atmospheric environment, µg/m³)
    PMS_FIELD_PM10_ATM,     // PM10 concentration (atmospheric environment, µg/m³)
    PMS_FIELD_PC_0_3,       // Number of particles >0.3 µm in 0.1 L air
    PMS_FIELD_PC_0_5,       // Number of particles >0.5 µm in 0.1 L air
    PMS_FIELD_PC_1_0,       // Number of particles >1.0 µm in 0.1 L air
    PMS_FIELD_PC_2_5,       // Number of particles >2.5 µm in 0.1 L air
    PMS_FIELD_PC_5_0,       // Number of particles >5.0 µm in 0.1 L air
    PMS_FIELD_PC_10,        // Number of particles >10 µm in 0.1 L air
    PMS_FIELD_TEMP,         // Temperature (PMS5003T and PMS5003ST)
    PMS_FIELD_HUMIDITY,     // Humidity (PMS5003T and PMS5003ST)
    PMS_FIELD_MAX
} pms_field_t;

/**
 * @brief PMS config struct
 */
typedef struct{
    pms_type_e type;
    gpio_num_t set_gpio;
    gpio_num_t reset_gpio;
    uart_port_t uart_port;
} pms_config_t;

/**
 * @brief PMS sensor struct
 */
typedef struct{
    pms_type_e type;
    pms_state_e state;
    pms_mode_e mode;
    uart_port_t uart_port;
    gpio_num_t set_gpio;
    gpio_num_t reset_gpio;
    uint8_t raw_data[32];
    TimerHandle_t reset_timer;
} pms_sensor_t;

/**
 * @brief Initialize PMS sensor
 *
 * @param pms_config PMS config struct
 * @retval ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if GPIO pins are invalid
 * @retval ESP_ERR_NO_MEM if reset timer creation failed
 * @retval ESP_FAIL if sensor fails to reset
 */
esp_err_t pms_init(pms_config_t *pms_config);

/**
 * @brief Deinitialize PMS sensor
 * 
 * @return ESP_OK on success 
 */
esp_err_t pms_deinit(void);

/**
 * @brief Reset PMS sensor
 * 
 * @retval ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor is in sleep state
 * @retval ESP_FAIL if sensor fails to change states
 * 
 * @note It takes at least 1.5s - 2s after reset before sensor can receive commands
 */
esp_err_t pms_reset(void);

/**
 * @brief Set PMS sensor state
 *
 * @param state State to put PMS sensor in
 * @retval ESP_OK on success
 * @retval ESP_ERR_INVALID_ARG if state is RESETING, STABILIZING or invalid
 * @retval ESP_ERR_INVALID_STATE if sensor is not in correct state to change to desired state
 * @retval ESP_ERR_INVALID_RESPONSE if send command fails or timer fails to start/change period
 */
esp_err_t pms_set_state(pms_state_e state);

/**
 * @brief Set PMS mode
 * 
 * @param mode Mode to put PMS sensor in
 * @retval ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor is in sleep state
 * @retval ESP_ERR_INVALID_ARG if mode is invalid
 * @retval ESP_ERR_INVALID_RESPONSE if send command fails
 */
esp_err_t pms_set_mode(pms_mode_e mode);

/**
 * @brief Get PMS state
 * 
 * @return Current state of PMS sensor 
 */
pms_state_e pms_get_state(void);

/**
 * @brief Get PMS mode
 * 
 * @return Current mode of PMS sensor 
 */
pms_mode_e pms_get_mode(void);

/**
 * @brief Get PMS UART port
 * 
 * @return UART port used by PMS sensor
 */
uart_port_t pms_get_uart_port(void);

/**
 * @brief Get PMS sensor type
 * 
 * @return PMS sensor type 
 */
pms_type_e pms_get_type(void);

/**
 * @brief Send command to read PMS data in passive mode
 * 
 * @return ESP_OK on success
 */
esp_err_t pms_send_passive_read_cmd(void);

/**
 * @brief Parse and verify PMS command response
 * 
 * @param data Raw data frame
 * @param len Data frame length
 * @return esp_err_t 
 */
esp_err_t pms_parse_cmd_response(const uint8_t *data, uint8_t len);

/**
 * @brief Parse PMS data frame
 *
 * @param data Raw data frame
 * @param len Data frame length
 * @retval ESP_OK on success
 * @retval ESP_ERR_INVALID_STATE if sensor is not in active state or passive mode
 * @retval ESP_FAIL if send command fails
 */
esp_err_t pms_parse_data(const uint8_t *data, uint8_t len);

/**
 * @brief Get data field from parsed PMS data frame
 *
 * @param field PMS filed ID
 * @return Data from parsed PMS frame
 */
int16_t pms_get_data(pms_field_t field);


#ifdef __cplusplus
}
#endif

#endif /* PMS_H_*/