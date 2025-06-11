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
 * Uses FreeRTOS tasks for NTC, LED, and potentiometer, and UART for command input.
 */

#include <stdio.h>
#include <math.h>
#include <string.h> // For strcpy, sscanf

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Your custom component headers
#include "adc_utils.h"
#include "tim_ch_duty.h" // Modified header
#include "io_utils.h" // Contains init_uart and COMPARE_STRINGS macro

// ESP-IDF driver headers
#include "driver/uart.h"
#include "esp_log.h" // For ESP_LOGI (used by init_uart inside io_utils)

//-------------------ADC------------------------
#define NTC_ADC_CH ADC_CHANNEL_5 //IO 33
#define POT_ADC_CH ADC_CHANNEL_3 //IO 39
#define ADC_UNIT   ADC_UNIT_1
#define ADC_ATTEN  ADC_ATTEN_DB_12

#define NTC_DATA_TYPE true
#define POT_DATA_TYPE false

//-------------------RGB-------------------------
#define R_CHANNEL     LEDC_CHANNEL_0
#define G_CHANNEL     LEDC_CHANNEL_1
#define B_CHANNEL     LEDC_CHANNEL_2
#define R_IO          GPIO_NUM_27
#define G_IO          GPIO_NUM_26
#define B_IO          GPIO_NUM_25
#define LED_FREQUENCY 1000

//-----------------UART---------------------------
#define UART_PORT           UART_NUM_0
#define UART_BAUD_RATE      115200
#define UART_RX_BUFFER_SIZE 1024 // Buffer size for raw UART receive
#define UART_TX_BUFFER_SIZE 0    // No software TX buffer (direct write)
#define UART_DRIVER_QUEUE_SIZE 20 // Queue size for UART events from driver
#define UART_APP_QUEUE_SIZE    10 // Queue size for processed strings for the app

#define UART_TX_PIN GPIO_NUM_1 // Example: Pin GPIO 1 for TX
#define UART_RX_PIN GPIO_NUM_3 // Example: Pin GPIO 3 for RX


//------------structs config-------------------

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

//-------------------Global Variables-------------------------

// Queues
static QueueHandle_t adc_data_queue = NULL;
static QueueHandle_t uart_driver_queue = NULL;       // Queue for UART events directly from the driver
static QueueHandle_t uart_app_processing_queue = NULL; // Queue for processed strings for our app logic

// Mutex for shared ADC resource
SemaphoreHandle_t xMutex;

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

// Global variable for current parsed patron
patron_temp_t current_patron;

// Global array for temperature level ranges (Red, Green, Blue)
// Assuming Red is for highest temp, Green for middle, Blue for lowest
float temp_levels[3][2] = {
    {40.0f, 80.0f}, // Red: Lower bound, Upper bound
    {20.0f, 40.0f}, // Green: Lower bound, Upper bound
    {0.0f, 20.0f}   // Blue: Lower bound, Upper bound
};

// Define if your RGB LED is common anode or common cathode
const bool IS_RGB_COMMON_ANODE = true; // Set to 'true' for common anode, 'false' for common cathode


//--------------------Helper Functions---------------------

/**
 * @brief Parses a string into a patron_temp_t struct.
 * Expected format: [C]XX.X|YY.Y (e.g., [R]10.5|20.1)
 *
 * @param data The null-terminated string to parse.
 * @return A patron_temp_t struct with parsed values.
 */
patron_temp_t patron_level_temp(char *data) {
    patron_temp_t result_patron = {0}; // Initialize to zeros

    sscanf(data, "[%c]%f|%f",
           &result_patron.color,
           &result_patron.temp_inf,
           &result_patron.temp_sup);

    current_patron = result_patron; // Assign to global
    printf("Parsed: Color=%c, Temp Inf=%.1f, Temp Sup=%.1f\r\n",
           result_patron.color, result_patron.temp_inf, result_patron.temp_sup);
    return result_patron;
}

/**
 * @brief Fills the global temp_levels array based on the parsed patron.
 *
 * @param temp_levels_ptr A pointer to the global temp_levels 2D array.
 * @param patron The patron_temp_t struct containing the color and temperature ranges.
 */
void fill_temp_levels(float (*temp_levels_ptr)[2], patron_temp_t patron) {
    switch(patron.color){
        case 'R':
            temp_levels_ptr[0][0] = patron.temp_inf;
            temp_levels_ptr[0][1] = patron.temp_sup;
            printf("Updated Red range: %.1f - %.1f\r\n", temp_levels_ptr[0][0], temp_levels_ptr[0][1]);
            break;
        case 'G':
            temp_levels_ptr[1][0] = patron.temp_inf;
            temp_levels_ptr[1][1] = patron.temp_sup;
            printf("Updated Green range: %.1f - %.1f\r\n", temp_levels_ptr[1][0], temp_levels_ptr[1][1]);
            break;
        case 'B':
            temp_levels_ptr[2][0] = patron.temp_inf;
            temp_levels_ptr[2][1] = patron.temp_sup;
            printf("Updated Blue range: %.1f - %.1f\r\n", temp_levels_ptr[2][0], temp_levels_ptr[2][1]);
            break;
        default:
            printf("Warning: Unknown color '%c' received for temp levels. Not updating.\r\n", patron.color);
            break;
    }
}


//--------------------Tasks---------------------

void ntc_task(void *pvParameters) {
    (void)pvParameters; // Suppress unused parameter warning

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

void pot_task(void *pvParameters) {
    (void)pvParameters; // Suppress unused parameter warning

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

        printf("Potentiometer Vout: %.2fV\r\n", Vo_pot);
        pot_item.value = Vo_pot;

        xQueueSend(adc_data_queue, &pot_item, portMAX_DELAY); // Send the full struct
        xSemaphoreGive(xMutex); // Release mutex
        vTaskDelay(pdMS_TO_TICKS(30)); // Delay for 30ms
    }
}

void uart_event_task(void *pvParameters) {
    (void)pvParameters; // Suppress unused parameter warning

    uart_event_t event;
    // Buffer for raw UART data received from the driver
    uint8_t *temp_rx_buffer = (uint8_t *) malloc(UART_RX_BUFFER_SIZE + 1); // +1 for null terminator
    
    printf("UART event task started.\r\n");

    while (1) {
        // Wait for a UART event from the driver's queue
        xQueueReceive(uart_driver_queue, (void *)&event, portMAX_DELAY);

        // Assuming UART_DATA event for simplicity as requested
        int len = uart_read_bytes(UART_PORT, temp_rx_buffer, event.size, portMAX_DELAY);
        temp_rx_buffer[len] = '\0'; // Null-terminate the received string

        // Allocate memory for the string to be sent to the application's processing queue
        char *processed_string = (char *) malloc(len + 1);
        strcpy(processed_string, (const char *)temp_rx_buffer);
        // Send the pointer to the received string to your application's processing queue
        xQueueSend(uart_app_processing_queue, &processed_string, 0);
        printf("UART RX (raw): %s\r\n", processed_string); // Echo what was received
    }
    // Note: free(temp_rx_buffer) is omitted as the task runs indefinitely
}


//------------------State Machine Task---------------------

void handle_duty_task(void *pvParameters) {
    (void)pvParameters; // Suppress unused parameter warning

    adc_type_data_t received_item;
    float current_temperature = 0.0f;
    float current_pot_voltage = 0.0f;
    uint8_t raw_brightness_percent = 0; // 0-100% derived from pot, 0=dim, 100=bright
    led_color_state_t current_led_state = LED_STATE_NONE; // Initial state

    // Initialize all LEDs off (0% brightness)
    rgb_pwm_set_color(&led_rgb, &timer, 0, 0, 0, IS_RGB_COMMON_ANODE); 

    char *received_uart_string;

    while (1) {
        // Try to receive from ADC queue (non-blocking if not available)
        xQueueReceive(adc_data_queue, &received_item, 0);
        if(received_item.type == NTC_DATA_TYPE) {
            current_temperature = received_item.value;
            printf("Current Temp: %.2f °C\r\n", current_temperature);
        } else if(received_item.type == POT_DATA_TYPE) {
            current_pot_voltage = received_item.value;
            // Map potentiometer voltage (0-3.3V) to 0-100% brightness
            raw_brightness_percent = (uint8_t)((current_pot_voltage / 3.3) * 100.0f); 
            // Clamp brightness to 0-100%
            if (raw_brightness_percent > 100) raw_brightness_percent = 100;
            printf("Pot Voltage: %.2fV, Brightness: %d%%\r\n",
                   current_pot_voltage, raw_brightness_percent);
        }

        // Try to receive from UART app processing queue (non-blocking if not available)
        if (xQueueReceive(uart_app_processing_queue, &received_uart_string, 0) == pdPASS) {
            printf("Processing UART command: '%s'\r\n", received_uart_string);
            patron_temp_t new_patron = patron_level_temp(received_uart_string);
            
            fill_temp_levels(temp_levels, new_patron); // Update global temp_levels
            
            free(received_uart_string); // Free the memory allocated in uart_event_task
        }


        led_color_state_t new_led_state = LED_STATE_NONE; // Default to none

        // Determine new LED state based on temperature
        // Using exclusive ranges to avoid overlap issues
        if (current_temperature < temp_levels[2][1]) { // BLUE: Below upper limit of blue range (e.g., <20)
            new_led_state = LED_STATE_BLUE;
        } else if (current_temperature >= temp_levels[1][0] && current_temperature < temp_levels[1][1]) { // GREEN: Within green range (e.g., >=20 and <40)
            new_led_state = LED_STATE_GREEN;
        } else if (current_temperature >= temp_levels[0][0] && current_temperature <= temp_levels[0][1]) { // RED: Within red range (e.g., >=40 and <=80)
            new_led_state = LED_STATE_RED;
        } else {
            // Temperature is outside all defined ranges
            new_led_state = LED_STATE_NONE; // All LEDs off
            printf("Temperature %.2f °C is out of defined LED ranges. LEDs off.\r\n", current_temperature);
        }

        // Only update LEDs if color state changed OR brightness value changed
        // For simplicity, we'll assume any change in raw_brightness_percent or state implies an update.
        // A more robust check might compare against actual current brightness for each color channel.
        if (new_led_state != current_led_state || raw_brightness_percent != 
            (IS_RGB_COMMON_ANODE ? (100 - led_rgb.red.duty_percent) : led_rgb.red.duty_percent) // Example for red
            // This is a simplified check for brightness change; a more detailed check for all channels could be added.
        )
        {
            current_led_state = new_led_state; // Update the current state

            switch (current_led_state) {
                case LED_STATE_BLUE:
                    rgb_pwm_set_color(&led_rgb, &timer, 0, 0, raw_brightness_percent, IS_RGB_COMMON_ANODE); // R & G off (0% brightness), B adjusted
                    printf("LED State: BLUE, Brightness: %d%%\r\n", raw_brightness_percent);
                    break;
                case LED_STATE_GREEN:
                    rgb_pwm_set_color(&led_rgb, &timer, 0, raw_brightness_percent, 0, IS_RGB_COMMON_ANODE); // R & B off (0% brightness), G adjusted
                    printf("LED State: GREEN, Brightness: %d%%\r\n", raw_brightness_percent);
                    break;
                case LED_STATE_RED:
                    rgb_pwm_set_color(&led_rgb, &timer, raw_brightness_percent, 0, 0, IS_RGB_COMMON_ANODE); // G & B off (0% brightness), R adjusted
                    printf("LED State: RED, Brightness: %d%%\r\n", raw_brightness_percent);
                    break;
                case LED_STATE_NONE:
                default:
                    rgb_pwm_set_color(&led_rgb, &timer, 0, 0, 0, IS_RGB_COMMON_ANODE); // All off (0% brightness)
                    printf("LED State: NONE (off)\r\n");
                    break;
            }
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

    // Create UART driver event queue
    uart_driver_queue = xQueueCreate(UART_DRIVER_QUEUE_SIZE, sizeof(uart_event_t));
    printf("UART driver event queue created.\r\n");

    // Create application UART processing queue
    uart_app_processing_queue = xQueueCreate(UART_APP_QUEUE_SIZE, sizeof(char *));
    printf("Application UART processing queue created.\r\n");

    // Initialize the UART using the function from io_utils
    init_uart(UART_PORT, UART_BAUD_RATE, UART_TX_PIN, UART_RX_PIN, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, uart_driver_queue);
    printf("UART initialized.\r\n");


    // Create tasks
    xTaskCreate(ntc_task, "ntc_task", 4096, NULL, 4, NULL);
    xTaskCreate(pot_task, "pot_task", 4096, NULL, 4, NULL);
    xTaskCreate(uart_event_task, "uart_event_task", 8192, NULL, 5, NULL);
    xTaskCreate(handle_duty_task, "handle_duty_task", 4096, NULL, 3, NULL);

    printf("All tasks created.\r\n");
    printf("-------------------System Ready-------------------\r\n");
}