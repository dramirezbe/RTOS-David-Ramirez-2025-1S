/**
 * @file main.c
 * @author David Ramírez Betancourth
 * @details Controlar 1 led rgb, leer temp termistor cada 3seg, Botón que active desactive la impresión por consola de la temp (uart),
 * fijar los límites de la página, tanto por página como por uart (intentar hacer).
 */

#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/queue.h"

#include "adc_utils.h"
#include "tim_ch_duty.h"
#include "io_utils.h"

#include "wifi_app.h"
#include "http_server.h"

#include <math.h>

//-------------------ADC-----------------------
#define NTC_ADC_CH ADC_CHANNEL_5 //IO 33
#define POT_ADC_CH ADC_CHANNEL_3 //IO 39
#define ADC_UNIT   ADC_UNIT_1
#define ADC_ATTEN  ADC_ATTEN_DB_12

#define NTC_DATA_TYPE true
#define POT_DATA_TYPE false

//-------------------RGB DUTY-----------------------
#define RD_CHANNEL     LEDC_CHANNEL_0
#define GD_CHANNEL     LEDC_CHANNEL_1
#define BD_CHANNEL     LEDC_CHANNEL_2
#define RD_IO          GPIO_NUM_27
#define GD_IO          GPIO_NUM_26
#define BD_IO          GPIO_NUM_25

//-------------------RGB TEMP-----------------------
#define RT_CHANNEL     LEDC_CHANNEL_3
#define GT_CHANNEL     LEDC_CHANNEL_4
#define BT_CHANNEL     LEDC_CHANNEL_5
#define RT_IO          GPIO_NUM_13
#define GT_IO          GPIO_NUM_12
#define BT_IO          GPIO_NUM_14

#define LED_FREQUENCY 1000
#define IS_RGB_COMMON_ANODE false //both



//-------------------UART----------------------
#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define BLINK_GPIO 2

//-----------------------------------------Struct----------------------------------------
typedef struct {
    char color;
    float temp_inf;
    float temp_sup;
} patron_temp_t;

// Enum for LED color states
typedef enum {
    LED_STATE_BLUE,
    LED_STATE_GREEN,
    LED_STATE_RED,
    LED_STATE_NONE // For initial or off state, or if no conditions are met
} led_color_state_t;

typedef struct {
    float value;
    bool type;   // false pot, true ntc
} adc_type_data_t;

//---------------------------------------Global Vars ------------------------------------
QueueHandle_t rgb_event_queue;
QueueHandle_t uart_status_queue;
QueueHandle_t temp_uart_queue;
QueueHandle_t temp_http_queue;

static QueueHandle_t adc_data_queue;
static QueueHandle_t uart_rx_queue;

// Mutex for shared ADC resource
SemaphoreHandle_t xMutex;

bool uart_on = true;

static uint8_t uart_rx_buffer[RD_BUF_SIZE];

//------------------------------------Config Peripherals-------------------------------------

// LEDC Timer and Channels configuration
pwm_timer_config_t timer = {.frequency_hz = LED_FREQUENCY, .resolution_bit = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0};

rgb_pwm_t led_rgb_duty = {
    .red   = { .channel = RD_CHANNEL, .gpio_num = RD_IO, .duty_percent = 0 },
    .green = { .channel = GD_CHANNEL, .gpio_num = GD_IO, .duty_percent = 0 },
    .blue  = { .channel = BD_CHANNEL, .gpio_num = BD_IO, .duty_percent = 0 }
};

rgb_pwm_t led_rgb_temp = {
    .red   = { .channel = RT_CHANNEL, .gpio_num = RT_IO, .duty_percent = 0 },
    .green = { .channel = GT_CHANNEL, .gpio_num = GT_IO, .duty_percent = 0 },
    .blue  = { .channel = BT_CHANNEL, .gpio_num = BT_IO, .duty_percent = 0 }
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

//-----------------------------------Helper Functions------------------------------------------

patron_temp_t patron_level_temp(char *data) {
    patron_temp_t result_patron = {0}; //Initialize

    sscanf(data, "[%c]%f|%f",
           &result_patron.color,
           &result_patron.temp_inf,
           &result_patron.temp_sup);

    //VERBOSE
    printf("Parsed: Color=%c, Temp Inf=%.1f, Temp Sup=%.1f\r\n", result_patron.color, result_patron.temp_inf, result_patron.temp_sup);
    return result_patron;
}

// Change global array
float temp_levels[3][2] = {
    {40.0, 80.0}, // Default for Red (assuming >40)
    {20.0, 40.0}, // Default for Green (assuming 20-40)
    {0.0, 20.0}   // Default for Blue (assuming <20)
};

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

    //VERBOSE

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
    }
}

static void configure_blink_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	
}

double temperature;
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

        temperature = final_T; //Send to http as global var
        
        //VERBOSE
        //printf("T: %.2f °C\r\n", final_T);
        ntc_item.value = final_T;
        xQueueSend(adc_data_queue, &ntc_item, (TickType_t)pdMS_TO_TICKS(10)); // Send the full struct
        xSemaphoreGive(xMutex); // Release mutex

        
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 100ms
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


        //VERBOSE
        //printf("P: %.2fV\r\n", Vo_pot);
        pot_item.value = Vo_pot;

        xQueueSend(adc_data_queue, &pot_item, (TickType_t)pdMS_TO_TICKS(10)); // Send the full struct
        xSemaphoreGive(xMutex); // Release mutex
        
        vTaskDelay(pdMS_TO_TICKS(450)); // Delay for 30ms
    }
}

void rgb_pwm_task(void *pvParameters) {
	rgb_values_t rgb_values;

    rgb_pwm_set_color(&led_rgb_duty, &timer, 0, 0, 75, IS_RGB_COMMON_ANODE);

	while (1) {
		if(xQueueReceive(rgb_event_queue, &rgb_values, (TickType_t)pdMS_TO_TICKS(10))) {
			printf("Queue Received RGB values: Red=%d, Green=%d, Blue=%d\n", rgb_values.red_val, rgb_values.green_val, rgb_values.blue_val);

            rgb_pwm_set_color(&led_rgb_duty, &timer, rgb_values.red_val, rgb_values.green_val, rgb_values.blue_val, IS_RGB_COMMON_ANODE);

		}
        vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void rgb_temp_task(void *pvParameters) {    

    led_color_state_t current_led_state = LED_STATE_NONE; // Initial state
    patron_temp_t current_uart_patron;
    adc_type_data_t received_item;

    float current_ntc_temp = 0.0f;
    float current_pot_voltage = 0.0f;
    uint8_t raw_brightness_percent = 100;

    rgb_pwm_set_color(&led_rgb_temp, &timer, 0, 0, 75, IS_RGB_COMMON_ANODE);

    while (1) {

        //VERBOSE
        //printf("Thresholds: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\r\n", temp_levels[0][0], temp_levels[0][1], temp_levels[1][0], temp_levels[1][1], temp_levels[2][0], temp_levels[2][1]);
        
        // UPDATE UART TEMP THRESHOLDS
        if(xQueueReceive(temp_uart_queue, &current_uart_patron, (TickType_t)pdMS_TO_TICKS(10))) {
            fill_temp_levels(current_uart_patron);
        }  
        
        if(xQueueReceive(adc_data_queue, &received_item, (TickType_t)pdMS_TO_TICKS(10))) {
            if(received_item.type == NTC_DATA_TYPE) {

                //current_ntc_temp = received_item.value;
    
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
        }
        
        led_color_state_t new_led_state = LED_STATE_NONE; // Default to none
        current_ntc_temp = temperature;
        // Determine new LED state based on temperature
        // Check each color range using proper min/max thresholds

        //VERBOSE
        //printf("Current NTC temp: %.2f °C\r\n", current_ntc_temp);

        if (current_ntc_temp >= temp_levels[2][0] && current_ntc_temp <= temp_levels[2][1]) { // BLUE: Within blue range
            new_led_state = LED_STATE_BLUE;
        } else if (current_ntc_temp >= temp_levels[1][0] && current_ntc_temp <= temp_levels[1][1]) { // GREEN: Within green range
            new_led_state = LED_STATE_GREEN;
        } else if (current_ntc_temp >= temp_levels[0][0] && current_ntc_temp <= temp_levels[0][1]) { // RED: Within red range
            new_led_state = LED_STATE_RED;
        } else {
            // Temperature is outside all defined ranges
            new_led_state = LED_STATE_NONE; // All LEDs off
            printf("Temperature %.2f °C is out of defined LED ranges. LEDs off.\r\n", current_ntc_temp);
        }
        //printf("New LED state: %d\r\n", new_led_state);

        current_led_state = new_led_state; // Update the current state

        switch (current_led_state) {
            case LED_STATE_BLUE:
                rgb_pwm_set_color(&led_rgb_temp, &timer, 0, 0, raw_brightness_percent, IS_RGB_COMMON_ANODE); // R & G off (0% brightness), B adjusted
                //pwm_set_duty(&led_rgb_temp.blue, &timer, raw_brightness_percent);
                break;
            case LED_STATE_GREEN:
                rgb_pwm_set_color(&led_rgb_temp, &timer, 0, raw_brightness_percent, 0, IS_RGB_COMMON_ANODE); // R & B off (0% brightness), G adjusted
                //pwm_set_duty(&led_rgb_temp.green, &timer, raw_brightness_percent);
                break;
            case LED_STATE_RED:
                rgb_pwm_set_color(&led_rgb_temp, &timer, raw_brightness_percent, 0, 0, IS_RGB_COMMON_ANODE); // G & B off (0% brightness), R adjusted
                //pwm_set_duty(&led_rgb_temp.red, &timer, raw_brightness_percent);
                break;
            case LED_STATE_NONE:
                break;
            default:
                rgb_pwm_set_color(&led_rgb_temp, &timer, 0, 0, 0, IS_RGB_COMMON_ANODE); // All off (0% brightness)
                break;
        }        

        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for the state machine

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
    uart_driver_install(UART_NUM, RD_BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_rx_queue, 0);

    printf("UART initialized.\r\n");

    uart_event_t event;
    while(1) {
        //wait UART RX
        if(xQueueReceive(uart_rx_queue, (void * )&event, (TickType_t)pdMS_TO_TICKS(10))) {
        
            uart_read_bytes(UART_NUM, uart_rx_buffer, event.size, portMAX_DELAY);
            uart_rx_buffer[event.size] = '\0';

            char *str_buffer = (char *)uart_rx_buffer;

			patron_temp_t received_patron = patron_level_temp(str_buffer);
			xQueueSend(temp_uart_queue, &received_patron, (TickType_t)pdMS_TO_TICKS(10));
            
        }
    }
}

void app_main(void)
{

    //MUTEX
    xMutex = xSemaphoreCreateMutex();
    printf("Mutex created successfully.\r\n");

    //PWM
    pwm_timer_init(&timer);
    printf("Timer Initialized. \r\n");

    rgb_pwm_init(&led_rgb_temp, &timer); // Set IOs for PWM
    printf("PWM rgb_temp Initialized. \r\n");


    rgb_pwm_init(&led_rgb_duty, &timer); // Set IOs for PWM
    printf("PWM rgb_duty Initialized. \r\n");

    //ADC
    

	rgb_event_queue = xQueueCreate(3, sizeof(rgb_values_t));
	temp_uart_queue = xQueueCreate(3, 2048);
    uart_status_queue = xQueueCreate(1, sizeof(bool));
    adc_data_queue = xQueueCreate(10, sizeof(adc_type_data_t));


    // Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	configure_blink_led();
	// Start Wifi
	wifi_app_start();

	xTaskCreate(rgb_pwm_task, "rgb_pwm_task", 2048, NULL, 5, NULL);
    xTaskCreate(rgb_temp_task, "rgb_temp_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_rx_task, "uart_rx_task", 2048, NULL, 5, NULL);
    xTaskCreate(ntc_task, "ntc_task", 4096, NULL, 4, NULL);
    xTaskCreate(pot_task, "pot_task", 4096, NULL, 4, NULL);

}

