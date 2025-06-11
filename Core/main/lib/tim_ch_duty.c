/**
 * @file tim_ch_duty.c
 * @author David Ramírez Betancourth
 */
#include "tim_ch_duty.h"

//------------------------------------------------------------------------------
// Initialize LEDC timer
//------------------------------------------------------------------------------
void pwm_timer_init(const pwm_timer_config_t *timer_cfg)
{
    ledc_timer_config_t cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = timer_cfg->timer_num,
        .duty_resolution = timer_cfg->resolution_bit,
        .freq_hz         = timer_cfg->frequency_hz,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&cfg);
}

//------------------------------------------------------------------------------
// Initialize single PWM channel
//------------------------------------------------------------------------------
void pwm_channel_init(const pwm_channel_t *ch_cfg, const pwm_timer_config_t *timer_cfg)
{
    uint32_t raw_duty = (ch_cfg->duty_percent * ((1 << timer_cfg->resolution_bit) - 1)) / 100;
    ledc_channel_config_t cfg = {
        .speed_mode  = LEDC_LOW_SPEED_MODE,
        .channel     = ch_cfg->channel,
        .timer_sel   = timer_cfg->timer_num,
        .intr_type   = LEDC_INTR_DISABLE,
        .gpio_num    = ch_cfg->gpio_num,
        .duty        = raw_duty,
        .hpoint      = 0
    };
    ledc_channel_config(&cfg);
}

//------------------------------------------------------------------------------
// Initialize RGB LED PWM channels
//------------------------------------------------------------------------------
void rgb_pwm_init(const rgb_pwm_t *led, const pwm_timer_config_t *timer_cfg)
{
    pwm_timer_init(timer_cfg);
    pwm_channel_init(&led->red, timer_cfg);
    pwm_channel_init(&led->green, timer_cfg);
    pwm_channel_init(&led->blue, timer_cfg);
}

//------------------------------------------------------------------------------
// Set duty cycle for one PWM channel
//------------------------------------------------------------------------------
void pwm_set_duty(const pwm_channel_t *ch_cfg, const pwm_timer_config_t *timer_cfg, uint8_t duty_percent)
{
    uint32_t max_duty = (1 << timer_cfg->resolution_bit) - 1;
    uint32_t raw_duty = (duty_percent * max_duty) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch_cfg->channel, raw_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch_cfg->channel);
}

//------------------------------------------------------------------------------
// Set color for RGB LED
//------------------------------------------------------------------------------
// Modified: Logic to invert duty for common anode LEDs
void rgb_pwm_set_color(const rgb_pwm_t *led, const pwm_timer_config_t *timer_cfg,
                        uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent, bool is_common_anode)
{
    uint8_t actual_red_duty = red_percent;
    uint8_t actual_green_duty = green_percent;
    uint8_t actual_blue_duty = blue_percent;

    if (is_common_anode) {
        // For common anode, 0% brightness means 100% duty cycle, and vice-versa.
        // So, invert the percentage: desired_brightness_percent = 100 - original_duty_percent.
        actual_red_duty = 100 - red_percent;
        actual_green_duty = 100 - green_percent;
        actual_blue_duty = 100 - blue_percent;
    }

    pwm_set_duty(&led->red, timer_cfg, actual_red_duty);
    pwm_set_duty(&led->green, timer_cfg, actual_green_duty);
    pwm_set_duty(&led->blue, timer_cfg, actual_blue_duty);
}