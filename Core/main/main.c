/**
 * @file main.c
 * @author David Ramírez Betancourth
 * @details System with a web interface for controlling a servo motor and monitoring environmental data.
 *  It supports SSID and password configuration for both STA and AP Wi-Fi modes, with these credentials persisted in memory.
 *  The servo motor operates in three distinct modes: potentiometer control, a 2-state temperature threshold mode (using an NTC thermistor),
 *  and direct register control, with all servo configurations saved to memory.
 *  A web page displays real-time data including NTC temperature, servo opening percentage, light intensity (from LDR), potentiometer value, and the current time.
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
#include "tasks_common.h"

#include <math.h>

//-------------------ADC-----------------------
#define NTC_ADC_CH ADC_CHANNEL_7 //IO 35
#define POT_ADC_CH ADC_CHANNEL_6 //IO 34
#define PHOTORES_ADC_CH ADC_CHANNEL_4 //IO 32


#define ADC_UNIT   ADC_UNIT_1
#define ADC_ATTEN  ADC_ATTEN_DB_12

//-------------------PWM----------------------
#define PWM_FREQ_HZ 50
#define SERVO_PIN GPIO_NUM_27

//-----------------------------------------Struct----------------------------------------

typedef enum {
    POT_TYPE,
    NTC_TYPE,
    PHOTORES_TYPE
}adc_type_t;

typedef struct {
    float value;
    adc_type_t type;
} adc_type_data_t;


//---------------------------------------Global Vars ------------------------------------

static QueueHandle_t adc_data_queue;

QueueHandle_t http_send_servo_pwm_queue;
QueueHandle_t http_send_ntc_queue;
QueueHandle_t http_send_photores_queue;

QueueHandle_t http_receive_servo_ctl_mode;
QueueHandle_t http_receive_thres_values_queue;

//QueueHandle_t http_send_hour_queue;    //LAST TO IMPLEMENT..............................
//QueueHandle_t http_send_photores_queue;



// Mutex for shared ADC resource
SemaphoreHandle_t xMutex;

//------------------------------------Config Peripherals-------------------------------------

// LEDC Timer and Channels configuration
pwm_timer_config_t timer = {.frequency_hz = PWM_FREQ_HZ, .resolution_bit = LEDC_TIMER_10_BIT, .timer_num = LEDC_TIMER_0};

pwm_channel_t servo_pwm = {.channel = LEDC_CHANNEL_0, .gpio_num = SERVO_PIN, .duty_percent = 0};

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

adc_config_t photores_adc_conf = {
    .unit_id = ADC_UNIT,
    .channel = PHOTORES_ADC_CH,
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH_12,
};
adc_channel_handle_t photores_adc_handle = NULL;
//-----------------------------------Helper Functions------------------------------------------


void ntc_task(void *arg) {

    adc_type_data_t ntc_item;
    adc_type_t NTC_DATA_TYPE = NTC_TYPE;
    ntc_item.type = NTC_DATA_TYPE;
    ntc_item.value = 0;

    // These constants should be specific to your NTC thermistor
    const float beta = 10000.0;     // Beta value for your NTC
    const float R0 = 10000.0;      // Resistance of NTC at T0 (e.g., 10k Ohms at 25°C)
    const float T0 = 298.15;       // Reference temperature in Kelvin (25°C + 273.15)
    const float R2 = 1000.0;       // Resistor in voltage divider (1k Ohm)

    const float Vi = 4.8;          // Input voltage to the voltage divider

    float Vo = 0;                  // Voltage measured across R2
    float Rt = 0;                  // Resistance of the NTC
    float T = 0;                   // Temperature in Kelvin
    float final_T = 0;             // Temperature in Celsius

    // 1. Configure ADC
    set_adc(&ntc_adc_conf, &ntc_adc_handle); // Assuming ntc_adc_conf and photores_adc_handle are defined globally

    int raw_val = 0;
    int voltage_mv = 0;

    while (1) {

        xSemaphoreTake(xMutex, portMAX_DELAY); // Acquire mutex for ADC

        get_raw_data(ntc_adc_handle, &raw_val);
        raw_to_voltage(ntc_adc_handle, raw_val, &voltage_mv);

        Vo = (float)voltage_mv / 1000.0f; // Convert mV to Volts (voltage across R2)

        // CORRECTED FORMULA FOR Rt (NTC Resistance)
        // Given: Vo is measured across R2, R1 is NTC, Vi is total input
        // Rt = R2 * (Vi - Vo) / Vo
        // Make sure (Vi - Vo) is not negative or zero if Vo approaches Vi,
        // which implies issues with the circuit or ADC range.
        if (Vo > 0 && Vi > Vo) { // Avoid division by zero and negative resistance
            Rt = (R2 * (Vi - Vo)) / Vo;
        } else {
            // Handle error or set a default/error value for Rt
            // For now, we'll just skip the calculation and print an error.
            printf("Error: Invalid Vo (%.2fV) or Vi (%.2fV) for Rt calculation.\r\n", Vo, Vi);
            xSemaphoreGive(xMutex); // Release mutex
            vTaskDelay(pdMS_TO_TICKS(500));
            continue; // Skip to next iteration
        }


        // Steinhart-Hart equation (Beta approximation) - this part was already correct
        T = (beta * T0) / (logf(Rt / R0) * T0 + beta);
        final_T = T - 273.15f; // Convert Kelvin to Celsius

        // VERBOSE
        

        ntc_item.value = final_T;
        // Ensure the queue exists and is handled correctly by the consumer task
        if (xQueueSend(adc_data_queue, &ntc_item, (TickType_t)pdMS_TO_TICKS(10)) != pdPASS) {
            printf("Failed to send NTC data to queue.\r\n");
        }

        xSemaphoreGive(xMutex); // Release mutex

        vTaskDelay(pdMS_TO_TICKS(51)); // Delay
    }
}

void pot_task(void *arg) {
    
    adc_type_data_t pot_item;
    adc_type_t POT_ADC_DATA_TYPE = POT_TYPE;
    pot_item.type = POT_ADC_DATA_TYPE;
    pot_item.value = 0;

    // 1. Configure ADC
    set_adc(&pot_adc_conf, &pot_adc_handle);

    int raw_val = 0;
    int voltage_mv = 0;
    float voltage = 0;
    float percent_pwm_servo = 0;

    while (1) {
        
        xSemaphoreTake(xMutex, portMAX_DELAY);

        get_raw_data(pot_adc_handle, &raw_val);
        raw_to_voltage(pot_adc_handle, raw_val, &voltage_mv);

        voltage = (float)voltage_mv / 1000.0f;

        percent_pwm_servo = 10 * voltage;

        //VERBOSE   
        //printf("PWM [%.2f], V [%.2fV]\r\n", percent_pwm_servo, voltage);
        
        pot_item.value = percent_pwm_servo;
        xQueueSend(adc_data_queue, &pot_item, (TickType_t)pdMS_TO_TICKS(10));
        xSemaphoreGive(xMutex); // Release mutex

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void photores_task(void *arg) {

    adc_type_data_t photores_item;
    adc_type_t PHOTORES_DATA_TYPE = PHOTORES_TYPE;
    photores_item.type = PHOTORES_DATA_TYPE;
    photores_item.value = 0;

    const float Vi = 4.8;          // Input voltage to the voltage divider
    const float R1 = 100000.0;       // Resistor in voltage divider (100k Ohm)

    // 1. Configure ADC
    set_adc(&photores_adc_conf, &photores_adc_handle);

    int raw_val = 0;
    int voltage_mv = 0;
    float V = 0;
    float current_photores = 0;

    while (1) {

        xSemaphoreTake(xMutex, portMAX_DELAY); // Acquire mutex for ADC

        get_raw_data(photores_adc_handle, &raw_val);
        raw_to_voltage(photores_adc_handle, raw_val, &voltage_mv);

        V = (float)voltage_mv / 1000.0f; // Convert mV to Volts (voltage across R2)

        current_photores = (V * R1)/(Vi - V);

        if (!(current_photores <= 0)) { 

            photores_item.value = current_photores;
            xQueueSend(adc_data_queue, &photores_item, (TickType_t)pdMS_TO_TICKS(10));

        } else {
            printf("Error: Photores out of range. [%.2fV]\r\n", current_photores);
        }

        xSemaphoreGive(xMutex); // Release mutex

        vTaskDelay(pdMS_TO_TICKS(52)); // Delay
    }
}

void ctl_task(void *arg) {

    adc_type_data_t adc_item;
    float current_ntc = 0.0f;
    int current_pot = 0;
    float current_photores = 0.0f;

    float current_thres[2] = {19.0f, 22.0f};
    float new_thres[2];

    servo_ctl_mode_state_t current_servo_state = CTL_IDLE;
    servo_ctl_mode_state_t new_servo_state = CTL_IDLE;

    http_photores_data_t photores_data;
    day_state_t day_state = SUNNY;


    while(1) {

        // ADC catch
        if (xQueueReceive(adc_data_queue, &adc_item, 0) == pdPASS) {
            
            switch (adc_item.type) {
                case NTC_TYPE:
                    current_ntc = adc_item.value;
                    xQueueOverwrite(http_send_ntc_queue, &current_ntc);
                    break;

                case POT_TYPE:
                    current_pot = (int)adc_item.value;
                    xQueueOverwrite(http_send_servo_pwm_queue, &current_pot);
                    break;

                case PHOTORES_TYPE:
                    current_photores = adc_item.value;

                    if (current_photores < 1000) { // Includes current_photores <= 100
                        day_state = SUNNY;

                    } else if (current_photores < 10000) { // Includes current_photores >= 1000
                        day_state = CLOUDY;

                    } else if (current_photores < 100000) { // Includes current_photores >= 10000
                        day_state = AFTERNOON;

                    } else { // current_photores >= 100000
                        day_state = NIGHT;
                    }
                    
                    photores_data.day_state = day_state;
                    photores_data.current_photores = current_photores;
                    xQueueOverwrite(http_send_photores_queue, &photores_data);

                    break;
                default:
                    break;
            }
            
            
        }
        //THRES CATCH (HTTP)

        if (xQueueReceive(http_receive_thres_values_queue, &new_thres, 0) == pdPASS) {

            for (int i = 0; i < 2 ; i++) {
                current_thres[i] = new_thres[i];
            }

        }
        
        //HANDLE SERVO CONTROL STATE
        if (xQueueReceive(http_receive_servo_ctl_mode, &new_servo_state, 0) == pdPASS) {
            current_servo_state = new_servo_state;   
            printf("CURRENT CTL SERVO STATE: %d\r\n", current_servo_state);

        }

        //STATE MACHINE
        
        //VERBOSE
        printf("POT: %d, NTC: %.2f, THRES: [%.2f, %.2f], STATE: %d\r\n", current_pot, current_ntc, current_thres[0], current_thres[1], current_servo_state);

        switch(current_servo_state) {
            case CTL_POT:
                pwm_set_duty(&servo_pwm, &timer, current_pot);
                printf("SERV: %d\r\n", current_pot);                
                break;

            case CTL_THRES:
                // Condición 1: Temperatura por debajo del mínimo.
                if (current_ntc < current_thres[0]) {
                    pwm_set_duty(&servo_pwm, &timer, 1); // Servo en posición cerrada/mínima
                }
                // Condición 2: Temperatura ENTRE el mínimo y el máximo.
                else if (current_ntc >= current_thres[0] && current_ntc <= current_thres[1]) {
                    pwm_set_duty(&servo_pwm, &timer, 14); // Servo en posición abierta
                }
                // Condición 3: Todas las demás situaciones (temperatura por encima del máximo).
                else {
                    pwm_set_duty(&servo_pwm, &timer, 1); // Servo en otra posición (o cerrada)
                }
                break; // El break del case
            case CTL_REG: //Program this later
                break;

            case CTL_IDLE:
                pwm_set_duty(&servo_pwm, &timer, 0);

                break;

        }
        vTaskDelay(pdMS_TO_TICKS(20));
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

    pwm_channel_init(&servo_pwm, &timer);
    printf("Channel Initialized. \r\n");

    //QUEUES
    adc_data_queue = xQueueCreate(10, sizeof(adc_type_data_t));
    http_send_servo_pwm_queue = xQueueCreate(1, sizeof(int));
    http_send_ntc_queue = xQueueCreate(1, sizeof(float));
    http_send_photores_queue = xQueueCreate(1, sizeof(http_photores_data_t));
    http_receive_servo_ctl_mode = xQueueCreate(10, sizeof(servo_ctl_mode_state_t));
    http_receive_thres_values_queue = xQueueCreate(10, sizeof(float)*2);

    //Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);
	
	// Start Wifi
	wifi_app_start();

    xTaskCreate(ntc_task, "ntc_task", 4096, NULL, 4, NULL);
    xTaskCreate(pot_task, "pot_task", 4096, NULL, 4, NULL);
    xTaskCreate(ctl_task, "ctl_task", 4096, NULL, 4, NULL);
    xTaskCreate(photores_task, "photores_task", 4096, NULL, 4, NULL);

}