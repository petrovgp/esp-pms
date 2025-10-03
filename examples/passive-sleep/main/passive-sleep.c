#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "pms.h"

static const char *TAG = "pms example";

void pms_passive_task(void *pvParameters){    
    uint8_t data[128];
    
    while (true){
        pms_state_e state = pms_get_state();
        switch(state){
            case PMS_STATE_ACTIVE:
                if(pms_send_passive_read_cmd() == ESP_OK){
                    int len = uart_read_bytes(pms_get_uart_port(), data, sizeof(data), 100 / portTICK_PERIOD_MS);
                    if(len > 0){
                        if(pms_parse_data(data, len) == ESP_OK){
                            ESP_LOGI(TAG, "PM1 data: %d ug/m3", pms_get_data(PMS_FIELD_PM1_ATM));
                            ESP_LOGI(TAG, "PM2.5 data: %d ug/m3", pms_get_data(PMS_FIELD_PM2_5_ATM));
                            ESP_LOGI(TAG, "PM10 data: %d ug/m3", pms_get_data(PMS_FIELD_PM10_ATM));
                            ESP_LOGI(TAG, "putting sensor to sleep...");
                            if(pms_set_state(PMS_STATE_SLEEP) != ESP_OK){
                                ESP_LOGE(TAG, "failed to put sensor to sleep");
                            }
                        }
                    }else{
                        ESP_LOGW(TAG, "no data received from passive read command");
                    }
                }
                break;

            case PMS_STATE_SLEEP:
                ESP_LOGI(TAG, "sensor in sleep state, waking up in 5s...");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
                if(pms_set_state(PMS_STATE_ACTIVE) != ESP_OK){
                    ESP_LOGE(TAG, "failed to wake up sensor");
                }
                break;

            case PMS_STATE_RESETTING:
                ESP_LOGI(TAG, "sensor resetting, waiting...");
                break;

            case PMS_STATE_STABILIZING:
                if(pms_get_mode() != PMS_MODE_PASSIVE){
                    ESP_LOGI(TAG, "switching to passive mode...");
                    pms_set_mode(PMS_MODE_PASSIVE);
                    int len = uart_read_bytes(pms_get_uart_port(), data, sizeof(data), 100 / portTICK_PERIOD_MS);
                    if(len > 0){
                        pms_parse_cmd_response(data, len);
                    }else{
                        ESP_LOGW(TAG, "no response to mode command");
                    }
                }
                ESP_LOGI(TAG, "sensor stabilizing, waiting...");
                break;

            default:
                ESP_LOGE(TAG, "sensor in invalid state");
                break;
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void app_main(void){
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .stop_bits = UART_STOP_BITS_1,
        .parity = UART_PARITY_DISABLE,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    pms_config_t pms_config = {
        .type = PMS_TYPE_5003,
        .set_gpio = GPIO_NUM_9,
        .reset_gpio = GPIO_NUM_10,
        .uart_port = UART_NUM_0,
    };

    ESP_ERROR_CHECK(pms_init(&pms_config));

    xTaskCreate(pms_passive_task, "pms passive task", 4096, NULL, 11, NULL);
}
