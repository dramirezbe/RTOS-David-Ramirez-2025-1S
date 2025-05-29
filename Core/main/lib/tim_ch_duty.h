/**
 * @file tim_ch_duty.h 
 * @author David Ramírez Betancourth
 */
#ifndef TIM_CH_DUTY_H
#define TIM_CH_DUTY_H

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
// Data Structures
//------------------------------------------------------------------------------

/**
 * @brief Single PWM channel configuration
 */
typedef struct {
    ledc_channel_t channel;       ///< LEDC channel (0–7)
    gpio_num_t     gpio_num;      ///< GPIO pin number
    uint32_t       duty_percent;  ///< Initial duty cycle (0–100)
} pwm_channel_t;

/**
 * @brief RGB PWM grouping
 */
typedef struct {
    pwm_channel_t red;   ///< Red channel
    pwm_channel_t green; ///< Green channel
    pwm_channel_t blue;  ///< Blue channel
} rgb_pwm_t;

/**
 * @brief PWM timer configuration
 */
typedef struct {
    ledc_timer_t      timer_num;      ///< LEDC timer (0–3)
    uint32_t          frequency_hz;   ///< PWM frequency in Hz
    ledc_timer_bit_t  resolution_bit; ///< PWM resolution (1–16 bits)
} pwm_timer_config_t;


//------------------------------------------------------------------------------
// Initialization Functions
//------------------------------------------------------------------------------

void pwm_timer_init(const pwm_timer_config_t *timer_cfg);
void pwm_channel_init(const pwm_channel_t *ch_cfg, const pwm_timer_config_t *timer_cfg);
void rgb_pwm_init(const rgb_pwm_t *led, const pwm_timer_config_t *timer_cfg);

//------------------------------------------------------------------------------
// Duty Control Functions
//------------------------------------------------------------------------------

void pwm_set_duty(const pwm_channel_t *ch_cfg, const pwm_timer_config_t *timer_cfg, uint8_t duty_percent);
void rgb_pwm_set_color(const rgb_pwm_t *led, const pwm_timer_config_t *timer_cfg,
                         uint8_t red_percent, uint8_t green_percent, uint8_t blue_percent);


#ifdef __cplusplus
}
#endif

#endif // TIM_CH_DUTY_H