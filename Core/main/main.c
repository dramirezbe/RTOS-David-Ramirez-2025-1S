/**
 * @file main.c
 * @author David Ramírez Betancourth
 *         Santiago Bustamante Montoya
 * @brief Manages a 10k NTC thermistor (R1) in a 1k (R2) voltage divider.
 * @details Calculates temperature T using Rt=R0*exp[B(1/T-1/T0)] where R0=10k,
 * deriving Rt from Vo=Vi[R2/(Rt+R2)] -> Rt = R2(Vi+Vo)/Vo.
 * Then T = (beta*T0)/(Ln(Rt/R0)*T0 + beta).
 * Controls an RGB LED via a state machine (Blue <20°C, Green 20-40°C, Red >40°C).
 * A potentiometer adjusts the duty cycle (brightness) of the active color.
 *
 * Uses FreeRTOS tasks for NTC, LED, and potentiometer, and UART for command input.
 */


#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"

#include "adc_utils.h"
#include "tim_ch_duty.h"

//-------------------ADC-----------------------
#define NTC_ADC_CH ADC_CHANNEL_5 //IO 33
#define POT_ADC_CH ADC_CHANNEL_3 //IO 39
#define ADC_UNIT   ADC_UNIT_1
#define ADC_ATTEN  ADC_ATTEN_DB_12

#define NTC_DATA_TYPE true
#define POT_DATA_TYPE false

//-------------------RGB-----------------------
#define R_CHANNEL     LEDC_CHANNEL_0
#define G_CHANNEL     LEDC_CHANNEL_1
#define B_CHANNEL     LEDC_CHANNEL_2
#define R_IO          GPIO_NUM_27
#define G_IO          GPIO_NUM_26
#define B_IO          GPIO_NUM_25
#define LED_FREQUENCY 1000

const bool IS_RGB_COMMON_ANODE = true; // Set to 'true' for common anode, 'false' for common cathode

//-------------------UART----------------------
#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

//------------Define config-------------------
typedef struct {
    float value;
    bool type;   // false pot, true ntc
} adc_type_data_t;

// Enum for LED color states
typedef enum {
    LED_STATE_BLUE,
    LED_STATE_GREEN,
    LED_STATE_RED,
    LED_STATE_NONE // For initial or off state, or if no conditions are met
} led_color_state_t;

typedef struct {
    char color;
    float temp_inf;
    float temp_sup;
} patron_temp_t;

//---------------Global Variables--------------

static QueueHandle_t uart_queue;
static QueueHandle_t temp_changes_queue;
static QueueHandle_t adc_data_queue;

// Mutex for shared ADC resource
SemaphoreHandle_t xMutex;

static uint8_t uart_rx_buffer[RD_BUF_SIZE];

//Global temp threshold array
float temp_levels[3][2] = {
    {40.0f, 80.0f}, // R: min, max
    {20.0f, 40.0f}, // G: min, max
    {0.0f, 20.0f}   // B: min, max
};

// LEDC Timer and Channels configuration
pwm_timer_config_t timer = {.frequency_hz = LED_FREQUENCY, .resolution_bit = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0};

rgb_pwm_t led_rgb = {
    .red   = { .channel = R_CHANNEL, .gpio_num = R_IO, .duty_percent = 0 },
    .green = { .channel = G_CHANNEL, .gpio_num = G_IO, .duty_percent = 0 },
    .blue  = { .channel = B_CHANNEL, .gpio_num = B_IO, .duty_percent = 0 }
};

// ADC Configurations and Handles
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

//---------------Helper Functions------------

patron_temp_t patron_level_temp(char *data) {
    patron_temp_t result_patron = {0}; //Initialize

    sscanf(data, "[%c]%f|%f",
           &result_patron.color,
           &result_patron.temp_inf,
           &result_patron.temp_sup);

    //VERBOSE
    //printf("Parsed: Color=%c, Temp Inf=%.1f, Temp Sup=%.1f\r\n", result_patron.color, result_patron.temp_inf, result_patron.temp_sup);
    return result_patron;
}

// Change global array
void fill_temp_levels(patron_temp_t patron) {
    switch(patron.color){
        case 'R':
            temp_levels[0][0] = patron.temp_inf;
            temp_levels[0][1] = patron.temp_sup;
            printf("Updated Red range: %.1f - %.1f\r\n", temp_levels[0][0], temp_levels[0][1]);
            break;
        case 'G':
            temp_levels[1][0] = patron.temp_inf;
            temp_levels[1][1] = patron.temp_sup;
            printf("Updated Green range: %.1f - %.1f\r\n", temp_levels[1][0], temp_levels[1][1]);
            break;
        case 'B':
            temp_levels[2][0] = patron.temp_inf;
            temp_levels[2][1] = patron.temp_sup;
            printf("Updated Blue range: %.1f - %.1f\r\n", temp_levels[2][0], temp_levels[2][1]);
            break;
        default:
            printf("Error: Use it like: [Led]val|val\r\n");
            break;
    }

    /** VERBOSE
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {

            switch(i) {
                case 0:
                    printf("R-");
                    break;
                case 1:
                    printf("G-");
                    break;
                case 2:
                    printf("B-");
                    break;
            }
            switch(j) {
                case 0:
                    printf("min:");
                    break;
                case 1:
                    printf("max:");
                    break;
            }
            printf(" %.1f\r\n", temp_levels[i][j]);
        }
    }*/
}

//---------------Tasks-----------------


void ntc_task(void *arg) {
    
    adc_type_data_t ntc_item;
    ntc_item.type = NTC_DATA_TYPE;
    ntc_item.value = 0;

    const float beta = 3950.0;
    const float R0 = 10000.0;
    const float T0 = 298.15; // 25°C in Kelvin
    const float R2 = 1000.0; // Resistor in voltage divider

    const float Vi = 4.8; // Input voltage to the voltage divider
    float Vo = 0;
    float Rt = 0;
    float T = 0; // Temperature in Kelvin
    float final_T = 0; // Temperature in Celsius

    // 1. Configure ADC
    set_adc(&ntc_adc_conf, &ntc_adc_handle);

    int raw_val = 0;
    int voltage_mv = 0;

    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY); // Acquire mutex for ADC
        get_raw_data(ntc_adc_handle, &raw_val);
        raw_to_voltage(ntc_adc_handle, raw_val, &voltage_mv);

        Vo = (float)voltage_mv / 1000.0f; // Convert mV to Volts

        // Original formula for Rt
        Rt = (R2 * (Vi + Vo)) / Vo;

        // Original formula for T
        T = (beta * T0) / (logf(Rt / R0) * T0 + beta);
        final_T = T - 273.15f; // Convert Kelvin to Celsius

        printf("T: %.2f °C\r\n", final_T);
        ntc_item.value = final_T;
        xQueueSend(adc_data_queue, &ntc_item, portMAX_DELAY); // Send the full struct
        xSemaphoreGive(xMutex); // Release mutex
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms
    }
}

void pot_task(void *arg) {
    
    adc_type_data_t pot_item;
    pot_item.type = POT_DATA_TYPE;
    pot_item.value = 0;

    // 1. Configure ADC
    set_adc(&pot_adc_conf, &pot_adc_handle);

    int raw_val = 0;
    int voltage_mv_pot = 0;
    float Vo_pot = 0; // Voltage from potentiometer

    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY); // Acquire mutex for ADC
        // 2. Get Raw Data
        get_raw_data(pot_adc_handle, &raw_val);

        // 3. Convert to Voltage
        raw_to_voltage(pot_adc_handle, raw_val, &voltage_mv_pot);

        Vo_pot = (float)voltage_mv_pot / 1000.0f; // Convert mV to Volts

        printf("P: %.2fV\r\n", Vo_pot);
        pot_item.value = Vo_pot;

        xQueueSend(adc_data_queue, &pot_item, portMAX_DELAY); // Send the full struct
        xSemaphoreGive(xMutex); // Release mutex
        vTaskDelay(pdMS_TO_TICKS(30)); // Delay for 30ms
    }
}





// Read UART task
void uart_rx_task(void *arg) {
    //Config UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, RD_BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0);

    printf("UART initialized.\r\n");

    uart_event_t event;
    while(1) {
        //wait UART RX
        if(xQueueReceive(uart_queue, (void * )&event, 20)) {
        
            uart_read_bytes(UART_NUM, uart_rx_buffer, event.size, portMAX_DELAY);
            uart_rx_buffer[event.size] = '\0';

            char *str_buffer = (char *)uart_rx_buffer;
            patron_temp_t received_patron = patron_level_temp(str_buffer);

            xQueueSend(temp_changes_queue, &received_patron, (TickType_t)portMAX_DELAY);
            
        }
    }
}

void handle_duty_task(void *arg) {
    adc_type_data_t received_item;
    float current_ntc_temp = 0.0f;
    float current_pot_voltage = 0.0f;
    uint8_t raw_brightness_percent = 0;
    led_color_state_t current_led_state = LED_STATE_NONE; // Initial state

    patron_temp_t current_thr;

    // Initialize all LEDs off (0% brightness)
    rgb_pwm_set_color(&led_rgb, &timer, 0, 0, 0, IS_RGB_COMMON_ANODE);

    while(1) {
        // ADC TASKS
        xQueueReceive(adc_data_queue, &received_item, 0);

        if(received_item.type == NTC_DATA_TYPE) {

            current_ntc_temp = received_item.value;

            //VERBOSE
            //printf("Current Temp: %.2f °C\r\n", current_temperature);
        } else if(received_item.type == POT_DATA_TYPE) {

            current_pot_voltage = received_item.value;
            // Map potentiometer voltage (0-3.3V) to 0-100% brightness
            raw_brightness_percent = (uint8_t)((current_pot_voltage / 3.3) * 100.0f); 
            // Clamp brightness to 0-100%
            if (raw_brightness_percent > 100) raw_brightness_percent = 100;

            //VERBOSE
            //printf("Pot Voltage: %.2fV, Brightness: %d%%\r\n", current_pot_voltage, raw_brightness_percent);
        }

        // UPDATE TEMP THRESHOLDS
        if(xQueueReceive(temp_changes_queue, &current_thr, (TickType_t)pdMS_TO_TICKS(10))) {
            fill_temp_levels(current_thr);
        }

        led_color_state_t new_led_state = LED_STATE_NONE; // Default to none

        // Determine new LED state based on temperature
        // Using exclusive ranges to avoid overlap issues
        if (current_ntc_temp < temp_levels[2][1]) { // BLUE: Below upper limit of blue range (e.g., <20)
            new_led_state = LED_STATE_BLUE;
        } else if (current_ntc_temp >= temp_levels[1][0] && current_ntc_temp < temp_levels[1][1]) { // GREEN: Within green range (e.g., >=20 and <40)
            new_led_state = LED_STATE_GREEN;
        } else if (current_ntc_temp >= temp_levels[0][0] && current_ntc_temp <= temp_levels[0][1]) { // RED: Within red range (e.g., >=40 and <=80)
            new_led_state = LED_STATE_RED;
        } else {
            // Temperature is outside all defined ranges
            new_led_state = LED_STATE_NONE; // All LEDs off
            printf("Temperature %.2f °C is out of defined LED ranges. LEDs off.\r\n", current_ntc_temp);
        }

        current_led_state = new_led_state; // Update the current state

        switch (current_led_state) {
            case LED_STATE_BLUE:
                rgb_pwm_set_color(&led_rgb, &timer, 0, 0, raw_brightness_percent, IS_RGB_COMMON_ANODE); // R & G off (0% brightness), B adjusted
                break;
            case LED_STATE_GREEN:
                rgb_pwm_set_color(&led_rgb, &timer, 0, raw_brightness_percent, 0, IS_RGB_COMMON_ANODE); // R & B off (0% brightness), G adjusted
                break;
            case LED_STATE_RED:
                rgb_pwm_set_color(&led_rgb, &timer, raw_brightness_percent, 0, 0, IS_RGB_COMMON_ANODE); // G & B off (0% brightness), R adjusted
                break;
            case LED_STATE_NONE:
                break;
            default:
                rgb_pwm_set_color(&led_rgb, &timer, 0, 0, 0, IS_RGB_COMMON_ANODE); // All off (0% brightness)
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for the state machine
    }
}

void app_main(void) {

    xMutex = xSemaphoreCreateMutex();
    pwm_timer_init(&timer);
    rgb_pwm_init(&led_rgb, &timer); // Set IOs for PWM

    printf("Mutex created successfully.\r\n");

    // Create ADC data queue
    adc_data_queue = xQueueCreate(10, sizeof(adc_type_data_t));
    printf("ADC data queue created.\r\n");

    // Create the queue for temperature changes
    temp_changes_queue = xQueueCreate(10, sizeof(patron_temp_t)); // Queue can hold 10 patron_temp_t items

    // Create tasks
    xTaskCreate(ntc_task, "ntc_task", 4096, NULL, 4, NULL);
    xTaskCreate(pot_task, "pot_task", 4096, NULL, 4, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 10, NULL); // Increased stack size for safety
    xTaskCreate(handle_duty_task, "handle_duty_task", 2048, NULL, 5, NULL);

    printf("All tasks created.\r\n");
    printf("-------------------System Ready-------------------\r\n");
}