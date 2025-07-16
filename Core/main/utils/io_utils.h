/**
 * @file io_utils.h 
 * @author David Ramírez Betancourth
 */


#ifndef GPIO_LIB_H
#define GPIO_LIB_H

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"

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
 * @brief Macro para comparar dos strings.
 *
 * @param str1 Primer string a comparar.
 * @param str2 Segundo string a comparar.
 * @return true si los strings son idénticos, false en caso contrario.
 */
#define COMPARE_STRINGS(str1, str2) (strcmp((const char *)(str1), (const char *)(str2)) == 0)

#endif // GPIO_LIB_H