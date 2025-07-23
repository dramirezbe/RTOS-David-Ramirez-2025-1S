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