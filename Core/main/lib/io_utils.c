/**
 * @file io_utils.c
 * @author David Ramírez Betancourth
 * @brief GPIO config macros utils, contains isr configs also
 */

#include "io_utils.h"
#include "esp_log.h" // Still needed for ESP_ERROR_CHECK internally

void io_config(gpio_num_t io_num,
                         bool       is_input,
                         bool       pull_up,
                         bool       open_drain)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << io_num),
        .mode         = is_input
                             ? GPIO_MODE_INPUT
                             : (open_drain
                                  ? GPIO_MODE_OUTPUT_OD
                                  : GPIO_MODE_OUTPUT),
        .pull_up_en   = pull_up   ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_up   ? GPIO_PULLDOWN_DISABLE: GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);
}

void isr_io_config(gpio_num_t       io_num,
                                 bool           is_input,
                                 bool           pull_up,
                                 bool           open_drain,
                                 gpio_int_type_t isr_type)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << io_num),
        .mode         = is_input
                             ? GPIO_MODE_INPUT
                             : (open_drain
                                  ? GPIO_MODE_OUTPUT_OD
                                  : GPIO_MODE_OUTPUT),
        .pull_up_en   = pull_up   ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_up   ? GPIO_PULLDOWN_DISABLE: GPIO_PULLDOWN_ENABLE,
        .intr_type    = isr_type
    };

    gpio_config(&io_conf);
}


bool is_debounced(TickType_t current_time_ticks, TickType_t last_event_time_ticks, int debounce_time_ms) {
    return ((current_time_ticks - last_event_time_ticks) * portTICK_PERIOD_MS) >= debounce_time_ms;
}

void init_uart(uart_port_t uart_num, int baud_rate, int tx_pin, int rx_pin, int rx_buffer_size, int tx_buffer_size, QueueHandle_t uart_driver_queue) {
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122, // Default threshold for HW flow control if enabled
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // CORRECTED: Pass the address of the QueueHandle_t
    // The 4th argument (queue_size) needs to be the actual size of the queue you want
    // the driver to use. If uart_driver_queue is NULL, this value is ignored.
    // Assuming 10 as a default for driver's internal queue for events if a queue is provided.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, 10, &uart_driver_queue, 0));
}