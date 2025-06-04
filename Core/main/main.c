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
#include "freertos/semphr.h"

#include "adc_utils.h"
#include "tim_ch_duty.h"
#include "io_utils.h"
#include "driver/uart.h"

//-------------------ADC------------------------
#define NTC_ADC_CH   ADC_CHANNEL_5 //IO 33
#define POT_ADC_CH   ADC_CHANNEL_3 //IO 39
#define ADC_UNIT      ADC_UNIT_1              
#define ADC_ATTEN     ADC_ATTEN_DB_12

#define NTC_DATA_TYPE true
#define POT_DATA_TYPE false

//-------------------RGB-------------------------
#define R_CHANNEL         LEDC_CHANNEL_0
#define G_CHANNEL         LEDC_CHANNEL_1
#define B_CHANNEL         LEDC_CHANNEL_2
#define R_IO              GPIO_NUM_27
#define G_IO              GPIO_NUM_26
#define B_IO              GPIO_NUM_25
#define LED_FREQUENCY     1000

//-----------------UART---------------------------
#define UART_PORT          UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_RX_BUFFER_SIZE 256
#define UART_QUEUE_LENGTH  20 // Queue for UART events/data


//------------structs config-------------------

typedef struct {
    float value;
    bool type;   //flase pot, true ntc
}adc_type_data_t;

// Enum for LED color states
typedef enum {
    LED_STATE_BLUE,
    LED_STATE_GREEN,
    LED_STATE_RED,
    LED_STATE_NONE // For initial or off state, or if no conditions are met
} led_color_state_t;

//-------------------Global Variables-------------------------

static QueueHandle_t adc_data_queue = NULL;
static QueueHandle_t uart_queue = NULL;    // Queue for UART events

SemaphoreHandle_t xMutex;

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

//--------------------ADC tasks---------------------
void ntc_task(void *pvParameters) {
    (void)pvParameters; // Suppress unused parameter warning

    adc_type_data_t ntc_item;
    ntc_item.type = NTC_DATA_TYPE;
    ntc_item.value = 0;

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

        while (1) {
            if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
           
            get_raw_data(ntc_adc_handle, &raw_val);
            raw_to_voltage(ntc_adc_handle, raw_val, &voltage_mv);
            
            float Vo = voltage_mv / 1000.0;

            float Rt = (R2*(Vi+Vo))/Vo;

            float T = (beta*T0)/(log(Rt/R0)*T0 + beta);

            float final_T = T - 273.15;
            printf("T: %.2f °C\n------------------------\n", final_T);

            ntc_item.value = final_T;
            xQueueSend(adc_data_queue, &ntc_item, portMAX_DELAY);
            xSemaphoreGive(xMutex);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

float Vo_pot;

void pot_task(void *pvParameters) {
    (void)pvParameters; // Suppress unused parameter warning

    adc_type_data_t pot_item;
    pot_item.type = POT_DATA_TYPE;
    pot_item.value = 0;

    // 1. Configure ADC
    set_adc(&pot_adc_conf, &pot_adc_handle);

    int raw_val = 0;
    int voltage_mv_pot = 0;
    
    while (1)
    {
        if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
                // 2. Get Raw Data
            get_raw_data(pot_adc_handle, &raw_val);

            // 3. Convert to Voltage
            raw_to_voltage(pot_adc_handle, raw_val, &voltage_mv_pot);

            Vo_pot = voltage_mv_pot / 1000.0;

            printf("Vout: %.2fV\n------------------------\n", Vo_pot);
            pot_item.value = Vo_pot;

            xQueueSend(adc_data_queue, &Vo_pot, portMAX_DELAY);
            xSemaphoreGive(xMutex);

        }
        vTaskDelay(30 / portTICK_PERIOD_MS);       
    }

}

void uart_event_task(void *pvParameters)
{
    (void)pvParameters; // Suppress unused parameter warning
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(UART_RX_BUFFER_SIZE + 1);

    init_uart(UART_PORT, UART_BAUD_RATE, &uart_queue, UART_RX_BUFFER_SIZE);

    while(1) {
        if(xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            bzero(data, UART_RX_BUFFER_SIZE + 1); // Clear buffer

            uart_read_bytes(UART_PORT, data, event.size, portMAX_DELAY);
            printf("UART received: %.*s\n", event.size, data);
            //uart logic
        }
    }
    free(data);
}


//------------------State Machine---------------------

void handle_duty_task() {
    adc_type_data_t received_item;
    float current_temperature = 0.0f;
    float current_pot_voltage = 0.0f;
    uint8_t raw_brightness_percent = 0; // 0-100% derived from pot, 0=dim, 100=bright
    uint8_t inverted_duty_percent = 100; // 0-100%, 0=bright (anode), 100=dim (anode)
    led_color_state_t current_led_state = LED_STATE_NONE; // Initial state

    rgb_pwm_set_color(&led_rgb, &timer, 100, 100, 100); // Initial state: all off

    while (1)
    {
        if(xQueueReceive(adc_data_queue, &received_item, portMAX_DELAY) == pdTRUE)
        {
            if(received_item.type == NTC_DATA_TYPE) {
                current_temperature = received_item.value;
            
            } else if(received_item.type == POT_DATA_TYPE) {
                current_pot_voltage = received_item.value;
                raw_brightness_percent = (uint8_t)((current_pot_voltage / 4.8) * 100.0f);
                // Clamp brightness to 0-100%
                if (raw_brightness_percent > 100) raw_brightness_percent = 100;
                
                inverted_duty_percent = 100 - raw_brightness_percent;
            }

            led_color_state_t new_led_state;
            if (current_temperature < 20.0f) {
                new_led_state = LED_STATE_BLUE;
            } else if (current_temperature >= 20.0f && current_temperature < 40.0f) {
                new_led_state = LED_STATE_GREEN;
            } else { // current_temperature >= 40.0f
                new_led_state = LED_STATE_RED;
            }

            // Only update LEDs if state or brightness has changed significantly
            // This prevents continuous calls to set_color if values are stable.
            // A simple threshold could be added for brightness, but for now, any change triggers update.
            if (new_led_state != current_led_state || // Color state changed
                (received_item.type == POT_DATA_TYPE) // Or only potentiometer value changed, re-apply brightness
               )
            {
                current_led_state = new_led_state; // Update the current state

                switch (current_led_state) {
                    case LED_STATE_BLUE:
                        rgb_pwm_set_color(&led_rgb, &timer, 100, 100, inverted_duty_percent); // R & G off (100% duty), B adjusted
                        break;
                    case LED_STATE_GREEN:
                        rgb_pwm_set_color(&led_rgb, &timer, 100, inverted_duty_percent, 100); // R & B off (100% duty), G adjusted
                        break;
                    case LED_STATE_RED:
                        rgb_pwm_set_color(&led_rgb, &timer, inverted_duty_percent, 100, 100); // G & B off (100% duty), R adjusted
                        break;
                    case LED_STATE_NONE: // Should ideally not be reached if temp conditions always met
                    default:
                        rgb_pwm_set_color(&led_rgb, &timer, 100, 100, 100); // All off
                        break;
                }
            }

        }

    }
}

void app_main(void)
{
    xMutex = xSemaphoreCreateMutex();
    pwm_timer_init(&timer);
    rgb_pwm_init(&led_rgb, &timer); //set io's pwm

    if (xMutex != NULL) {
        printf("Mutex created successfully\n");

        adc_data_queue = xQueueCreate(10, sizeof(adc_type_data_t));
        uart_queue = xQueueCreate(UART_QUEUE_LENGTH, sizeof(uart_event_t)); // Queue for UART events

        xTaskCreate(ntc_task, "ntc_task", 4098, NULL, 4, NULL);
        xTaskCreate(pot_task, "pot_task", 4098, NULL, 4, NULL);
        xTaskCreate(uart_event_task, "uart_event_task", 4098, NULL, 5, NULL);
        xTaskCreate(handle_duty_task, "handle_duty_task", 4098, NULL, 3, NULL);

    } else {
        printf("Failed to create mutex\n");
        return;
    }    

    printf("-------------------Done-------------------");

}