#ifndef GPIO_LIB_H
#define GPIO_LIB_H

#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>

/**
 * @brief Configure a GPIO pin for input or output, with optional pull-up/down and open-drain.
 *
 * @param io_num      GPIO number (e.g. GPIO_NUM_0 … GPIO_NUM_39)
 * @param is_input    true→input mode; false→output mode
 * @param pull_up     true→enable pull-up; false→enable pull-down
 * @param open_drain  true→open-drain (only when output); false→push-pull
 */
void io_config(gpio_num_t io_num,
                        bool       is_input,
                        bool       pull_up,
                        bool       open_drain);

/**
 * @brief Same as gpio_lib_io_config(), plus attach an interrupt handler.
 *
 * @param io_num      GPIO number
 * @param is_input    true→input mode; false→output mode
 * @param pull_up     true→enable pull-up; false→enable pull-down
 * @param open_drain  true→open-drain (only when output); false→push-pull
 * @param isr_type    GPIO_INTR_… type for triggering
 */
void isr_io_config(gpio_num_t      io_num,
                            bool            is_input,
                            bool            pull_up,
                            bool            open_drain,
                            gpio_int_type_t isr_type);

/**
 * @brief Checks if a debounce time has passed since the last event.
 *
 * @param current_time_ticks The current FreeRTOS tick count.
 * @param last_event_time_ticks The FreeRTOS tick count of the last event.
 * @param debounce_time_ms The debounce time in milliseconds.
 * @return true If the debounce time has passed.
 * @return false Otherwise.
 */
bool is_debounced(TickType_t current_time_ticks, TickType_t last_event_time_ticks, int debounce_time_ms);

/**
 * @brief * @brief Initializes the UART with the specified parameters.
 *
 * @param port The UART port to initialize (e.g., UART_NUM_0, UART_NUM_1).
 * @param baud_rate The baud rate for the UART communication.
 * @param queue Pointer to the queue handle for UART events.
 * @param buffer_size The size of the UART buffer.
 */
void init_uart(uart_port_t port, uint32_t baud_rate, QueueHandle_t *queue, uint32_t buffer_size);

#endif // GPIO_LIB_H