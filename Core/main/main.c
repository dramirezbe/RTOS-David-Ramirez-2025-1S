/**
 * @file main.c
 * @author David Ramírez Betancourth
 * @brief Manages a 10k NTC thermistor (R1) in a 1k (R2) voltage divider.
 * @details Calculates temperature T using Rt=R0*exp[B(1/T-1/T0)] where R0=10k,
 * deriving Rt from Vo=Vi[R2/(Rt+R2)] -> Rt = R2(Vi+Vo)/Vo.
 * Then T = (beta*T0)/(Ln(Rt/R0)*T0 + beta).
 * Controls an RGB LED via a state machine (Blue <20°C, Green 20-40°C, Red >40°C).
 * A potentiometer adjusts the duty cycle (brightness) of the active color.
 * 
 * Use task for ntc, led and potentiometer
 */

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "adc_utils.h"
#include "tim_ch_duty.h"

//-------------------ADC------------------------
#define NTC_ADC_CH   ADC_CHANNEL_5 //IO 33
#define POT_ADC_CH   ADC_CHANNEL_3 //IO 39


#define ADC_UNIT      ADC_UNIT_1              


#define ADC_ATTEN     ADC_ATTEN_DB_12

//-------------------RGB-------------------------
#define R_CHANNEL         LEDC_CHANNEL_0
#define G_CHANNEL         LEDC_CHANNEL_1
#define B_CHANNEL         LEDC_CHANNEL_2
#define R_IO              GPIO_NUM_32
#define G_IO              GPIO_NUM_35
#define B_IO              GPIO_NUM_34
#define LED_FREQUENCY     1000

static QueueHandle_t gpio_evt_queue = NULL;

uint16_t duty_led_status[3] = {100,100,100};

pwm_timer_config_t timer = {.frequency_hz = 1000, .resolution_bit = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0};

rgb_pwm_t led_rgb = {
    .red   = { .channel = R_CHANNEL, .gpio_num = R_IO, .duty_percent = 0 },
    .green = { .channel = G_CHANNEL, .gpio_num = G_IO, .duty_percent = 0 },
    .blue  = { .channel = B_CHANNEL, .gpio_num = B_IO, .duty_percent = 0 }
};

adc_config_t ntc_adc_conf = {
    .unit_id = ADC_UNIT,
    .channel = NTC_ADC_CH,
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_12,
};
adc_channel_handle_t ntc_adc_handle = NULL;

adc_config_t pot_adc_conf = {
    .unit_id = ADC_UNIT,
    .channel = POT_ADC_CH,
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_12,
};
adc_channel_handle_t pot_adc_handle = NULL;


void ntc_task() {

    const float beta = 3950.0;
    const float R0 = 10000.0;
    const float T0 =  298.15;
    const float R2 = 1000.0;

    const float Vi = 4.8;
    float Vo = 0; 
    float Rt = 0;
    float T = 0;

    // 1. Configure ADC
    set_adc(&ntc_adc_conf, &ntc_adc_handle);

    int raw_val = 0;
    int voltage_mv = 0;
    
    while (1)
    {
        // 2. Get Raw Data
        get_raw_data(ntc_adc_handle, &raw_val);

        // 3. Convert to Voltage
        raw_to_voltage(ntc_adc_handle, raw_val, &voltage_mv);
        
        float Vo = voltage_mv / 1000.0;
        printf("Vout: %.2fV\t", Vo);
        float Rt = (R2*(Vi+Vo))/Vo;
        printf("Rt: %.2f\t", Rt);

        float T = (beta*T0)/(log(Rt/R0)*T0 + beta);
        printf("T: %.2f °C\n------------------------\n", T - 273.15);

        xQueueSend();

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

float Vo_pot;

void pot_task() {
    // 1. Configure ADC
    set_adc(&pot_adc_conf, &pot_adc_handle);

    int raw_val = 0;
    int voltage_mv_pot = 0;
    
    while (1)
    {
        // 2. Get Raw Data
        get_raw_data(pot_adc_handle, &raw_val);

        // 3. Convert to Voltage
        raw_to_voltage(pot_adc_handle, raw_val, &voltage_mv_pot);

        Vo_pot = voltage_mv_pot / 1000.0;

        vTaskDelay(1000 / portTICK_PERIOD_MS);       
    }

}


void handle_duty_task() {
    while (1)
    {
        if(xQueueReceive())
        {
            

        }

    }
}

void app_main(void)
{
    pwm_timer_init(&timer);
    rgb_pwm_init(&led_rgb, &timer); //set io's pwm
    rgb_pwm_set_color(&led_rgb, &timer, duty_led_status[0], duty_led_status[1], duty_led_status[2]); //Initial

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(ntc_task, "ntc_task", 2048, NULL, 4, NULL);
    xTaskCreate(pot_task, "ntc_task", 2048, NULL, 4, NULL);

    printf("-------------------Done-------------------");

}