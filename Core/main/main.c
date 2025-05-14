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
#include "driver/gpio.h"
#include "esp_err.h"
#include "gpio_handler.h"   // Assumed to provide gpio_three_channel_init and set_three_channel_duty 

typedef enum {
    IDLE,         // (128,128,128)
    PURPLE_STATE, // (128,0,128)
    YELLOW_STATE, // (255,222,89)
    RED_STATE,    // (255,0,0)
    GREEN_STATE,  // (0,255,0)
    BLUE_STATE    // (0,0,255)
} color_state_t;

typedef struct {
    int r_val;
    int g_val;
    int b_val;
} rgb_state_t;

void actual_color_change(color_state_t state, rgb_state_t *rgb_change) {
    switch (state) {
        case IDLE:
            rgb_change->r_val = 128;
            rgb_change->g_val = 128;
            rgb_change->b_val = 128;
            break;
        case PURPLE_STATE:
            rgb_change->r_val = 128;
            rgb_change->g_val = 0;
            rgb_change->b_val = 128;
            break;
        case YELLOW_STATE:
            rgb_change->r_val = 255;
            rgb_change->g_val = 222;
            rgb_change->b_val = 89;
            break;
        case RED_STATE:
            rgb_change->r_val = 255;
            rgb_change->g_val = 0;
            rgb_change->b_val = 0;
            break;
        case GREEN_STATE:
            rgb_change->r_val = 0;
            rgb_change->g_val = 255;
            rgb_change->b_val = 0;
            break;
        case BLUE_STATE:
            rgb_change->r_val = 0;
            rgb_change->g_val = 0;
            rgb_change->b_val = 255;
            break;
        default:
            break;
    }
}

#define PIN_R           GPIO_NUM_27  
#define PIN_G           GPIO_NUM_26   
#define PIN_B           GPIO_NUM_25
#define BOARD_BUTTON    GPIO_NUM_0
#define DEBOUNCE_TIME_MS  50

// Queue to notify GPIO events
static QueueHandle_t gpio_evt_queue = NULL;

// State machine variable (initially IDLE)
static volatile color_state_t current_state = IDLE;

// Global RGB state variable; initialize to IDLE settings.
static rgb_state_t rgb_state = {128, 128, 128};

// Variable for debouncing
static TickType_t last_press_time = 0;

/**
 * @brief ISR for the button.
 *
 * Sends the GPIO number to the queue to be processed.
 */
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

/**
 * @brief Task that processes button press events.
 *
 * It debounces input and changes the LED’s color by cycling through the
 * available states.
 */
static void gpio_button_task(void* arg)
{
    uint32_t io_num; // GPIO number
    
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount();
            
            // Debounce check
            if (((now - last_press_time) * portTICK_PERIOD_MS) >= DEBOUNCE_TIME_MS) {
                last_press_time = now;
                
                // Cycle through states in order
                switch (current_state) {
                    case IDLE:
                        current_state = PURPLE_STATE;
                        break;
                    case PURPLE_STATE:
                        current_state = YELLOW_STATE;
                        break;
                    case YELLOW_STATE:
                        current_state = RED_STATE;
                        break;
                    case RED_STATE:
                        current_state = GREEN_STATE;
                        break;
                    case GREEN_STATE:
                        current_state = BLUE_STATE;
                        break;
                    case BLUE_STATE:
                        current_state = IDLE;
                        break;
                    default:
                        current_state = IDLE;
                        break;
                }
                
                // Update the current color based on the state
                actual_color_change(current_state, &rgb_state);
                
                // Use the GPIO‑RGB PWM library to update the LED
                mode_rgb_t mode = ANODE;
                units_duty_t units = RGB_DUTY;
                set_three_channel_duty(rgb_state.r_val, rgb_state.g_val, rgb_state.b_val, mode, units);
            }
        }
    }
}

void app_main(void)
{
    // Initialize PWM channels for the RGB LED.
    gpio_three_channel_init(PIN_R, PIN_G, PIN_B);

    mode_rgb_t mode = ANODE;
    units_duty_t units = RGB_DUTY;
    
    // Set initial LED color (IDLE state)
    actual_color_change(current_state, &rgb_state);
    set_three_channel_duty(rgb_state.r_val, rgb_state.g_val, rgb_state.b_val, mode, units);
    
    // Configure the button GPIO
    gpio_config_t io_conf = {0};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BOARD_BUTTON);
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    
    // Install GPIO ISR service and add the button handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOARD_BUTTON, gpio_isr_handler, (void*) BOARD_BUTTON);
    
    // Create the queue to pass GPIO events
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Create a task to handle button events
    xTaskCreate(gpio_button_task, "gpio_button_task", 2048, NULL, 10, NULL);
    
    // Main loop: nothing to do here, tasks manage the LED.
    while (true) {
        vTaskDelay(portMAX_DELAY);
    }
}
