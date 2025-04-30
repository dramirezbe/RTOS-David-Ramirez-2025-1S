/**
 * @authors 
 *         David Ramírez Betancourth,
 *         Santiago Bustamante Montoya
 *
 * @brief Utiliza interrupciones en BOARD_BUTTON (flanco de bajada, pull-up) para
 *        cambiar entre estados de operación:
 *        1) blink de 1 seg (1 sec ON, 1 sec OFF)
 *        2) blink de 0.5 seg (0.5 sec ON, 0.5 sec OFF)
 *        3) IDLE (LED apagado)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define BOARD_BUTTON      GPIO_NUM_0
#define BOARD_LED         GPIO_NUM_2
#define DEBOUNCE_TIME_MS  50

typedef enum {
    IDLE,          // LED apagado
    ONESEC_BLINK,  // blink de 1 seg
    HALFSEC_BLINK  // blink de 0.5 seg
} state_machine_t;


// Cola para notificar eventos
static QueueHandle_t gpio_evt_queue = NULL;

//máquina de estados (inicialmente IDLE)
static volatile state_machine_t current_state = IDLE;

/**
 * @brief Manejador de interrupción para el botón.
 *
 * Envía el número de GPIO a la cola para que la tarea correspondiente lo procese.
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief Procesa las interrupciones del botón.
 *
 * Cuando se detecta un evento en la cola, se utiliza un `switch` para actualizar
 * el estado del sistema de forma secuencial:
 *    IDLE -> ONESEC_BLINK -> HALFSEC_BLINK -> IDLE
 */
static void gpio_button_task(void* arg)
{
    uint32_t io_num; //gpio_num
    TickType_t last_press_time = 0;
    
    while(1) {

        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount(); //Systick

            //Debounce
            if (((now - last_press_time) * portTICK_PERIOD_MS) >= DEBOUNCE_TIME_MS) {
                last_press_time = now;

                // Cambiar el estado, dada la cola
                switch (current_state) {
                    case IDLE:
                        current_state = ONESEC_BLINK;
                        printf("Cambio a estado: ONESEC_BLINK\n");
                        break;
                    case ONESEC_BLINK:
                        current_state = HALFSEC_BLINK;
                        printf("Cambio a estado: HALFSEC_BLINK\n");
                        break;
                    case HALFSEC_BLINK:
                        current_state = IDLE;
                        // Se asegura de apagar el LED al pasar a IDLE
                        gpio_set_level(BOARD_LED, 0);
                        printf("Cambio a estado: IDLE (LED apagado)\n");
                        break;
                    default:
                        current_state = IDLE;
                        break;
                }
            }
        }
    }
}


/**
 * @brief Tarea LED.
 *
 * Dependiendo del estado, el LED se enciende y apaga con los intervalos
 * correspondientes. Cuando el estado es IDLE, se mantiene apagado.
 */
static void led_blink_task(void* arg)
{
    while(1) {
        switch (current_state) {
            case ONESEC_BLINK:
                gpio_set_level(BOARD_LED, 1);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                gpio_set_level(BOARD_LED, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            case HALFSEC_BLINK:
                gpio_set_level(BOARD_LED, 1);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                gpio_set_level(BOARD_LED, 0);
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;
            case IDLE:
            default:
                gpio_set_level(BOARD_LED, 0);

                // Eetardo para evitar bloqueo excesivo
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;
        }
    }
}

void app_main(void)
{
    // Configuración LED
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BOARD_LED),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // Configuración botón
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BOARD_BUTTON);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);

    // Crear la cola
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    // Crear tareas
    xTaskCreate(gpio_button_task, "gpio_button_task", 2048, NULL, 10, NULL);
    xTaskCreate(led_blink_task, "led_blink_task", 2048, NULL, 10, NULL);

    //Configurar interrupciones
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_EDGE | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(BOARD_BUTTON, gpio_isr_handler, (void*)BOARD_BUTTON);

    printf("-----------------Ready---------------");
}