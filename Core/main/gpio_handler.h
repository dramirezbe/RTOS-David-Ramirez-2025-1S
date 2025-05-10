#ifndef GPIO_HANDLER_H
#define GPIO_HANDLER_H

#include <stdint.h>
#include "driver/ledc.h"
#include "esp_err.h"

/**
 * @brief LED connection mode.
 *
 * - ANODE:    Common‑anode LED (duty is inverted: a low duty value → high brightness).
 * - CATHODE:  Common‑cathode LED (duty value is used directly).
 */
typedef enum {
    ANODE,
    CATHODE
} mode_rgb_t;

/**
 * @brief Units for specifying duty‑cycle input values.
 *
 * - PERCENTAGE_DUTY: Values from 0 to 100 (%).
 * - DECIMAL_DUTY:    Values from 0.0 to 1.0 (float).
 * - RGB_DUTY:        Values from 0 to 255 (8‑bit color).
 */
typedef enum {
    PERCENTAGE_DUTY,
    DECIMAL_DUTY,
    RGB_DUTY
} units_duty_t;

/**
 * @brief Initialize three‑channel PWM for an RGB LED.
 *
 * This function configures the LEDC timer (frequency, resolution, speed mode)
 * and then attaches three PWM channels to the specified GPIO pins, starting
 * with the LED off (duty = 0).
 *
 * @param red_gpio   GPIO number for the red channel.
 * @param green_gpio GPIO number for the green channel.
 * @param blue_gpio  GPIO number for the blue channel.
 *
 * @note Uses:
 *   - LEDC_TIMER_0,
 *   - LEDC_LOW_SPEED_MODE,
 *   - 13‑bit resolution,
 *   - 4 kHz PWM frequency.
 */
void gpio_three_channel_init(uint8_t red_gpio, uint8_t green_gpio, uint8_t blue_gpio);


/**
 * @brief Set RGB LED brightness by specifying new duty‑cycle values.
 *
 * Converts each input duty value (percentage, decimal or 8‑bit) into the
 * internal 13‑bit range (0 … 8191), inverts it if in ANODE mode, and updates
 * each PWM channel.
 *
 * @param red_duty     Duty input for red channel (see @p units).
 * @param green_duty   Duty input for green channel.
 * @param blue_duty    Duty input for blue channel.
 * @param mode         LED connection mode: ANODE or CATHODE.
 * @param units        Unit of the duty inputs: PERCENTAGE_DUTY,
 *                     DECIMAL_DUTY, or RGB_DUTY.
 *
 * @note If @p mode is ANODE, brightness increases as duty → 0.
 */
void set_three_channel_duty(int red_duty,
                            int green_duty,
                            int blue_duty,
                            mode_rgb_t mode,
                            units_duty_t units);

#endif // GPIO_HANDLER_H
