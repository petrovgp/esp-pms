#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/uart.h>
#include <esp_log.h>

#include <pms.h>

static const char *TAG = "pms example";

QueueHandle_t data_queue;

void pms_queue_task(void *pvParameters){
    uint8_t data[128];
    uart_event_t event;

    while (true){
        if (xQueueReceive(data_queue, (void *)&event, 100 / portTICK_PERIOD_MS)){
            switch(event.type){
                case UART_DATA:
                    int len = uart_read_bytes(pms_get_uart_port(), (uint8_t*)&data, sizeof(data), 0);
                    if(pms_get_state() == PMS_STATE_ACTIVE){
                        if(len > 0){
                            if(pms_parse_data((uint8_t*)&data, len) == ESP_OK){
                                ESP_LOGI(TAG, "PM1 data: %d ug/m3", pms_get_data(PMS_FIELD_PM1_ATM));
                                ESP_LOGI(TAG, "PM2.5 data: %d ug/m3", pms_get_data(PMS_FIELD_PM2_5_ATM));
                                ESP_LOGI(TAG, "PM10 data: %d ug/m3", pms_get_data(PMS_FIELD_PM10_ATM));
                            }
                        }
                    }else{
                        ESP_LOGW(TAG, "data received while sensor not in active state, ignoring");
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "hw fifo overflow");
                    uart_flush_input(pms_get_uart_port());
                    xQueueReset(data_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "ring buffer full");
                    uart_flush_input(pms_get_uart_port());
                    xQueueReset(data_queue);
                    break;

                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
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

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024 * 2, 0, 10, &data_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    pms_config_t pms_config = {
        .type = PMS_TYPE_5003,
        .set_gpio = GPIO_NUM_9,
        .reset_gpio = GPIO_NUM_10,
        .uart_port = UART_NUM_0,
    };

    ESP_ERROR_CHECK(pms_init(&pms_config));

    xTaskCreate(pms_queue_task, "pms queue task", 4096, NULL, 11, NULL);
}