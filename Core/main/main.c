
#include "tim_ch_duty.h"
#include "io_utils.h"

#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"

#define R_CHANNEL         LEDC_CHANNEL_0
#define G_CHANNEL         LEDC_CHANNEL_1
#define B_CHANNEL         LEDC_CHANNEL_2
#define R_IO              GPIO_NUM_3
#define G_IO              GPIO_NUM_4
#define B_IO              GPIO_NUM_5
#define LED_FREQUENCY     1000

#define BTN_R                   GPIO_NUM_25
#define BTN_G                   GPIO_NUM_33                   
#define BTN_B                   GPIO_NUM_32
#define DEBOUNCE_TIME_MS 40

#define ADD_DUTY 25

static QueueHandle_t gpio_evt_queue = NULL;

uint16_t duty_led_status[3] = {100,100,100};

pwm_timer_config_t timer = {.frequency_hz = 1000, .resolution_bit = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0};

/**
 * One way to do it for each io
 * 
pwm_channel_t pwm_r = {.channel = R_CHANNEL, .duty_percent = 0, .gpio_num = R_IO};
pwm_channel_t pwm_g = {.channel = G_CHANNEL, .duty_percent = 0, .gpio_num = G_IO};
pwm_channel_t pwm_b = {.channel = B_CHANNEL, .duty_percent = 0, .gpio_num = B_IO};
*/

rgb_pwm_t led_rgb = {
    .red   = { .channel = R_CHANNEL, .gpio_num = R_IO, .duty_percent = 0 },
    .green = { .channel = G_CHANNEL, .gpio_num = G_IO, .duty_percent = 0 },
    .blue  = { .channel = B_CHANNEL, .gpio_num = B_IO, .duty_percent = 0 }
};


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void btn_task(void* arg)
{
    uint32_t io_num; //gpio_num
    TickType_t last_press_time = 0;
    
    while(1) {

        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount(); //Systick

            //Debounce
            if (is_debounced(now, last_press_time, DEBOUNCE_TIME_MS)) {
                last_press_time = now;

                //Logic
                switch(io_num) {
                    case BTN_R:
                    duty_led_status[0] = duty_led_status[0] + ADD_DUTY;
                    printf("Led R duty: %d\n", duty_led_status[0]);
                        break;
                    case BTN_G:
                    duty_led_status[1] = duty_led_status[1] + ADD_DUTY;
                    printf("Led G duty: %d\n", duty_led_status[1]);
                        break;
                    case BTN_B:
                    duty_led_status[2] = duty_led_status[2] + ADD_DUTY;
                    printf("Led B duty: %d\n", duty_led_status[2]);
                        break;
                }

                for (int i = 0; i < 3; i++) {
                    if (duty_led_status[i] >= 100) {
                        duty_led_status[i] = 0;
                    }
                }
                rgb_pwm_set_color(&led_rgb, &timer, duty_led_status[0], duty_led_status[1], duty_led_status[2]);
            }
        }
    }
}


void app_main(void) {

    pwm_timer_init(&timer);
    rgb_pwm_init(&led_rgb, &timer); //set io's pwm
    rgb_pwm_set_color(&led_rgb, &timer, duty_led_status[0], duty_led_status[1], duty_led_status[2]); //Initial

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM);

    //gpio_num_t io_num, bool is_input, bool pull_up, bool open_drain
    isr_io_config(BTN_R, true, true, false, GPIO_INTR_NEGEDGE);
    isr_io_config(BTN_G, true, true, false, GPIO_INTR_NEGEDGE);
    isr_io_config(BTN_B, true, true, false, GPIO_INTR_NEGEDGE);
    
    
    gpio_isr_handler_add(BTN_R, gpio_isr_handler, (void*) BTN_R);
    gpio_isr_handler_add(BTN_G, gpio_isr_handler, (void*) BTN_G);
    gpio_isr_handler_add(BTN_B, gpio_isr_handler, (void*) BTN_B);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreatePinnedToCore(btn_task, "btn_task", 2048, NULL, 4, NULL, 0);

    printf("-------------------Done-------------------");
}