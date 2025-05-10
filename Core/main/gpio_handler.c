#include <stdio.h>
#include "gpio_handler.h"
#include "driver/ledc.h"
#include "esp_err.h"

// LEDC timer and channel configuration constants
#define LEDC_TIMER              LEDC_TIMER_0           /**< Hardware timer index */
#define LEDC_MODE               LEDC_LOW_SPEED_MODE    /**< Low‑speed PWM mode */
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT      /**< 13‑bit resolution (0…8191) */
#define LEDC_FREQUENCY          4000                   /**< PWM frequency: 4 kHz */

#define LEDC_CHANNEL_R          LEDC_CHANNEL_0         /**< Red channel index */
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1         /**< Green channel index */
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2         /**< Blue channel index */

void gpio_three_channel_init(uint8_t red_gpio, uint8_t green_gpio, uint8_t blue_gpio)
{
    // 1) Configure PWM timer
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    // 2) Prepare common channel configuration (start with LED off)
    ledc_channel_config_t chan_cfg = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 0,
        .hpoint         = 0
    };

    // Attach red channel
    chan_cfg.channel  = LEDC_CHANNEL_R;
    chan_cfg.gpio_num = red_gpio;
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));

    // Attach green channel
    chan_cfg.channel  = LEDC_CHANNEL_G;
    chan_cfg.gpio_num = green_gpio;
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));

    // Attach blue channel
    chan_cfg.channel  = LEDC_CHANNEL_B;
    chan_cfg.gpio_num = blue_gpio;
    ESP_ERROR_CHECK(ledc_channel_config(&chan_cfg));
}

void set_three_channel_duty(int red_duty,
                            int green_duty,
                            int blue_duty,
                            mode_rgb_t mode,
                            units_duty_t units)
{
    // Maximum internal duty value (2^13‑1 = 8191)
    const uint32_t max_duty = (1u << LEDC_DUTY_RES) - 1u;
    float scale = 100.0f;

    // Determine scale factor based on units
    switch (units) {
        case PERCENTAGE_DUTY: scale = 100.0f; break;
        case DECIMAL_DUTY:    scale = 1.0f;   break;
        case RGB_DUTY:        scale = 255.0f; break;
        default:              scale = 100.0f; break;
    }

    // Convert to 13‑bit duty
    uint32_t duty_r = (uint32_t)((red_duty   * max_duty) / scale);
    uint32_t duty_g = (uint32_t)((green_duty * max_duty) / scale);
    uint32_t duty_b = (uint32_t)((blue_duty  * max_duty) / scale);

    // Invert for common‑anode wiring
    if (mode == ANODE) {
        duty_r = max_duty - duty_r;
        duty_g = max_duty - duty_g;
        duty_b = max_duty - duty_b;
    }

    // Apply to each LEDC channel and update
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_R, duty_r));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_R));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_G, duty_g));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_G));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_B, duty_b));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_B));
}
