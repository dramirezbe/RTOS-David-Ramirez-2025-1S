/**
 * @file    main.c
 * @brief   Demonstrates basic usage of the GPIO‑RGB PWM library.
 *
 * This example performs the following steps:
 *  1. Initializes three PWM channels for an RGB LED (pins defined below).
 *  2. Sets the LED to a mid‑level blue color using percentage units.
 *  3. Leaves the LED in that state indefinitely.
 *
 * You can adapt the calls to `set_three_channel_duty()` to display any color
 * or brightness level (in percentage, decimal, or 0–255 RGB units) and switch
 * between ANODE/CATHODE wiring modes.
 *
 * @author  David Ramírez Betancourth
 * @author  Santiago Bustamante Montoya
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "gpio_handler.h"


#define PIN_R   GPIO_NUM_27  
#define PIN_G   GPIO_NUM_26   
#define PIN_B   GPIO_NUM_25   

void app_main(void)
{

    gpio_three_channel_init(PIN_R, PIN_G, PIN_B);

    mode_rgb_t    mode  = ANODE;
    units_duty_t  units = PERCENTAGE_DUTY;

    
    set_three_channel_duty(0, 0, 50, mode, units);

    
    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}
