/**
 * @file adc_utils.c
 * @author David Ramírez Betancourth
 * @brief ADC config macros utils, also with raw to voltage functions
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "adc_utils.h"

// Maximum number of ADC channels to support
#define MAX_ADC_CHANNELS 10

// Internal structure (matches the handle type)
struct adc_channel_handle_internal_t {
    adc_config_t config;
    adc_oneshot_unit_handle_t unit_handle;
    adc_cali_handle_t cali_handle;
    bool calibrated;
    bool in_use;
};

// Global handles and pool
static adc_oneshot_unit_handle_t adc_unit_handles[ADC_UNIT_2 + 1] = {NULL, NULL};
static struct adc_channel_handle_internal_t adc_channel_pool[MAX_ADC_CHANNELS];


static bool adc_calibration_init_internal(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    bool calibrated = false;

    if (!out_handle) {
        return false;
    }
    *out_handle = NULL;

    adc_cali_line_fitting_config_t cali_config_line = {
        .unit_id = unit,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    // Attempt to create the calibration scheme
    if (adc_cali_create_scheme_line_fitting(&cali_config_line, &handle) == ESP_OK) {
        calibrated = true;
    }

    if (calibrated) {
        *out_handle = handle;
    }

    return calibrated;
}


void set_adc(const adc_config_t *config, adc_channel_handle_t *out_handle)
{
    *out_handle = NULL;

    if (!config || !out_handle) {
        return;
    }

    int free_index = -1;
    for (int i = 0; i < MAX_ADC_CHANNELS; i++) {
        if (!adc_channel_pool[i].in_use) {
            free_index = i;
            break;
        }
    }

    if (free_index == -1) {
        return;
    }

    struct adc_channel_handle_internal_t *handle_data = &adc_channel_pool[free_index];
    memset(handle_data, 0, sizeof(struct adc_channel_handle_internal_t));
    handle_data->config = *config;

    // --- ADC Unit Init ---
    if (!adc_unit_handles[config->unit_id]) {
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = config->unit_id,
        };
        adc_oneshot_new_unit(&init_config, &adc_unit_handles[config->unit_id]);
    }
    handle_data->unit_handle = adc_unit_handles[config->unit_id];

    // --- ADC Channel Config ---
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = config->atten,
        .bitwidth = config->bitwidth,
    };
    adc_oneshot_config_channel(handle_data->unit_handle, config->channel, &chan_config);

    // --- ADC Calibration Init ---
    handle_data->calibrated = adc_calibration_init_internal(config->unit_id, config->channel, config->atten, &handle_data->cali_handle);

    handle_data->in_use = true;
    *out_handle = handle_data;
}


void get_raw_data(adc_channel_handle_t handle, int *out_raw)
{
    *out_raw = -1;

    if (!handle || !out_raw || !handle->in_use) {
        return;
    }

    adc_oneshot_read(handle->unit_handle, handle->config.channel, out_raw);
}


void raw_to_voltage(adc_channel_handle_t handle, int raw_data, int *out_voltage)
{
    *out_voltage = -1;

    if (!handle || !out_voltage || !handle->in_use) {
        return;
    }

    if (!handle->calibrated || !handle->cali_handle) {
        return;
    }

    adc_cali_raw_to_voltage(handle->cali_handle, raw_data, out_voltage);
}