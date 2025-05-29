/**
 * @file io_utils.c
 * @author David Ramírez Betancourth
 * @brief GPIO config macros utils, contains isr configs also
 */

#include "io_utils.h"

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

void isr_io_config(gpio_num_t      io_num,
                            bool            is_input,
                            bool            pull_up,
                            bool            open_drain,
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

void init_uart(uart_port_t port, uint32_t baud_rate, QueueHandle_t *queue, uint32_t buffer_size) {

    uart_config_t config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(port, &config);
    uart_driver_install(port, buffer_size, buffer_size, 10, queue, 0); // Pass the queue pointer directly
}
