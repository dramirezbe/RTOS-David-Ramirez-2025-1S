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
#include "driver/ledc.h"
#include "esp_err.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT     // Resolución de 13 bits
#define LEDC_FREQUENCY          4000                  // 4 kHz

// Pines GPIO para cada canal RGB
#define LEDC_OUTPUT_IO_R        GPIO_NUM_13   // Rojo
#define LEDC_OUTPUT_IO_G        GPIO_NUM_12   // Verde
#define LEDC_OUTPUT_IO_B        GPIO_NUM_14   // Azul

// Canales LEDC
#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2

static void example_ledc_init(void)
{
    // 1) Configurar el timer PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2) Configurar cada canal (R, G, B)
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 0,      // Arrancamos con 0% (apagado)
        .hpoint         = 0
    };

    // Rojo
    ledc_channel.channel  = LEDC_CHANNEL_R;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_R;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Verde
    ledc_channel.channel  = LEDC_CHANNEL_G;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_G;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Azul
    ledc_channel.channel  = LEDC_CHANNEL_B;
    ledc_channel.gpio_num = LEDC_OUTPUT_IO_B;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue)
{
    // Rango de duty: 0 … (2^13 − 1)
    uint32_t max_duty = (1 << LEDC_DUTY_RES) - 1;

    // Rojo
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, (red   * max_duty) / 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R));

    // Verde
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, (green * max_duty) / 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G));

    // Azul
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, (blue  * max_duty) / 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B));
}

void app_main(void)
{
    // Inicializar LEDC (timer + canales)
    example_ledc_init();

    // MORADO = Rojo 50%, Verde 0%, Azul 50%
    set_rgb_color(128, 0, 128);
    printf("----------------Ready----------------")

}
