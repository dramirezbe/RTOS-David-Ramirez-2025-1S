/**
 * @file adc_utils.h
 * @author David Ramírez Betancourth
 * @brief adc utils macros, header
 */


#ifndef ADC_UTILS_H
#define ADC_UTILS_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


/**
 * @brief ADC configuration structure
 */
typedef struct {
    adc_unit_t unit_id;          /*!< ADC unit ID */
    adc_channel_t channel;       /*!< ADC channel */
    adc_atten_t atten;           /*!< ADC attenuation */
    adc_bitwidth_t bitwidth;    /*!< ADC bitwidth */
} adc_config_t;

/**
 * @brief Opaque handle for an ADC channel configuration
 * (It's actually a pointer, but treated as opaque)
 */
typedef struct adc_channel_handle_internal_t* adc_channel_handle_t;

/**
 * @brief Configure and initialize an ADC channel.
 *
 * Initializes the ADC unit (if needed) and configures the channel.
 * It attempts calibration and stores the results.
 *
 * @param[in]  config      Pointer to the ADC configuration structure.
 * @param[out] out_handle  Pointer to store the created ADC channel handle.
 * Will be NULL if configuration fails.
 */
void set_adc(const adc_config_t *config, adc_channel_handle_t *out_handle);

/**
 * @brief Get the raw ADC reading from a configured channel.
 *
 * @param[in]  handle     Handle to the configured ADC channel.
 * @param[out] out_raw    Pointer to store the raw ADC value.
 * Set to -1 on failure.
 */
void get_raw_data(adc_channel_handle_t handle, int *out_raw);

/**
 * @brief Convert raw ADC reading to voltage (mV).
 *
 * Uses calibration if available.
 *
 * @param[in]  handle      Handle to the configured ADC channel.
 * @param[in]  raw_data    The raw ADC value to convert.
 * @param[out] out_voltage Pointer to store the voltage in mV.
 * Set to -1 if not calibrated.
 */
void raw_to_voltage(adc_channel_handle_t handle, int raw_data, int *out_voltage);


#endif // ADC_UTILS_H