/*!*****************************************************************
 * \file    adc.c
 * \brief   Common ADC driver based on LL driver.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "adc.h"

#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#include "stm32wl3x.h"
#include "stm32wl3x_ll_adc.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_pwr.h"

/*** ADC local macros ***/

#define ADC_INSTANCE                ADC1

#define ADC_VREF_TS_DELAY_MS        2

#define ADC_VMCU_MV_DEFAULT         3300
#define ADC_TMCU_DEGREES_DEFAULT    25

#define ADC_TIMEOUT_COUNT           1000000

#define ADC_CALIBRATION_POINT_NONE  0xFFFFFFFF

/*** ADC local structures ***/

/*******************************************************************/
typedef struct {
    uint32_t channel;
    uint32_t range;
    uint32_t calibration_point;
    uint32_t gain;
    uint32_t offset;
} ADC_conversion_t;

/*** ADC local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
static MCAL_status_t _ADC_single_conversion(ADC_conversion_t *conversion, uint32_t *adc_data_12bits) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    uint32_t tmp_gain;
    // Configure channel.
    LL_ADC_SetSequencerRanks(ADC_INSTANCE, LL_ADC_RANK_1, (conversion->channel));
    LL_ADC_SetChannelVoltageRange(ADC_INSTANCE, (conversion->channel), (conversion->range));
    // Set gain.
    if ((conversion->calibration_point) != ADC_CALIBRATION_POINT_NONE) {
        // Check if gain is explicitly given.
        if ((conversion->gain) != 0) {
            tmp_gain = (conversion->gain);
        }
        else if ((conversion->range) == LL_ADC_VIN_RANGE_1V2) {
            tmp_gain = 0xFFF;
        }
        else if ((conversion->range) == LL_ADC_VIN_RANGE_2V4) {
            tmp_gain = 0x7FF;
        }
        else {
            tmp_gain = 0x555;
        }
        LL_ADC_ConfigureCalibPoint(ADC_INSTANCE, (conversion->calibration_point), tmp_gain, (conversion->offset));
        // Attach calibration point to channel.
        switch (conversion->channel) {
        case LL_ADC_CHANNEL_VINM0:
        case LL_ADC_CHANNEL_VINM1:
        case LL_ADC_CHANNEL_VINM2:
        case LL_ADC_CHANNEL_VINM3:
            LL_ADC_SetCalibPointForSingleNeg(ADC_INSTANCE, (conversion->calibration_point), (conversion->range));
            break;
        case LL_ADC_CHANNEL_VINP0:
        case LL_ADC_CHANNEL_VINP1:
        case LL_ADC_CHANNEL_VINP2:
        case LL_ADC_CHANNEL_VINP3:
            LL_ADC_SetCalibPointForSinglePos(ADC_INSTANCE, (conversion->calibration_point), (conversion->range));
            break;
        case LL_ADC_CHANNEL_VINP0_VINM0:
        case LL_ADC_CHANNEL_VINP1_VINM1:
        case LL_ADC_CHANNEL_VINP2_VINM2:
        case LL_ADC_CHANNEL_VINP3_VINM3:
            LL_ADC_SetCalibPointForDiff(ADC_INSTANCE, (conversion->calibration_point), (conversion->range));
            break;
        case LL_ADC_CHANNEL_VBAT:
            LL_ADC_SetCalibPointForSingleNeg(ADC_INSTANCE, (conversion->calibration_point), LL_ADC_VIN_RANGE_3V6);
            break;
        case LL_ADC_CHANNEL_TEMPSENSOR:
            LL_ADC_SetCalibPointForSinglePos(ADC_INSTANCE, (conversion->calibration_point), LL_ADC_VIN_RANGE_1V2);
            break;
        default:
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Clear flag.
    LL_ADC_ClearFlag_EOS(ADC_INSTANCE);
    // Start conversion.
    LL_ADC_StartConversion(ADC_INSTANCE);
    // Wait for conversion to complete.
    while (LL_ADC_IsActiveFlag_EOS(ADC_INSTANCE) == 0) {
        // Wait end of conversion ('EOC='1') or timeout.
        loop_count++;
        if (loop_count > ADC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Read data.
    (*adc_data_12bits) = LL_ADC_DSGetOutputData(ADC_INSTANCE);
errors:
    return status;
}

/*** ADC functions ***/

/*******************************************************************/
MCAL_status_t ADC_init(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    LL_ADC_InitTypeDef adc_config;
    // Enable peripheral clock.
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_ADCANA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_ADCDIG);
    // Init ADC.
    adc_config.ContinuousConvMode = DISABLE;
    adc_config.Overrun = LL_ADC_NEW_DATA_IS_LOST;
    adc_config.SampleRate = LL_ADC_SAMPLE_RATE_140;
    adc_config.SamplingMode = LL_ADC_SAMPLING_AT_END;
    adc_config.SequenceLength = 1;
    ll_status = LL_ADC_Init(ADC_INSTANCE, &adc_config);
    _check_ll_status();
    // Turn ADC on.
    LL_ADC_Enable(ADC_INSTANCE);
    // Additional configuration.
    LL_ADC_ConfigureDSDataOutput(ADC_INSTANCE, LL_ADC_DS_DATA_WIDTH_12_BIT, LL_ADC_DS_RATIO_1);
    LL_ADC_InvertOutputSingleNegModeEnable(ADC_INSTANCE);
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t ADC_de_init(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    // Release ADC.
    ll_status = LL_ADC_DeInit(ADC_INSTANCE);
    _check_ll_status();
errors:
    // Turn ADC off
    LL_ADC_Disable(ADC_INSTANCE);
    // Disable peripheral clock.
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_ADCANA);
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_ADCDIG);
    return status;
}

/*******************************************************************/
MCAL_status_t ADC_get_mcu_voltage(uint16_t *mcu_voltage_mv) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ADC_conversion_t adc_conversion;
    uint32_t gain_pos_3v6 = 0;
    uint32_t offset_pos_3v6 = 0;
    uint32_t mcu_voltage_12bits = 0;
    // Check parameter.
    if (mcu_voltage_mv == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Reset result to default.
    (*mcu_voltage_mv) = ADC_VMCU_MV_DEFAULT;
    // Check if conversion data is available.
    gain_pos_3v6 = LL_ADC_GET_CALIB_GAIN_FOR_VINPX_3V6();
    offset_pos_3v6 = LL_ADC_GET_CALIB_OFFSET_FOR_VINPX_3V6();
    if (gain_pos_3v6 == 0xFFF) {
        // Set default values.
        gain_pos_3v6 = LL_ADC_DEFAULT_RANGE_VALUE_3V6;
        offset_pos_3v6 = 0;
    }
    // Configure conversion.
    adc_conversion.channel = LL_ADC_CHANNEL_VBAT;
    adc_conversion.range = LL_ADC_VIN_RANGE_3V6;
    adc_conversion.calibration_point = LL_ADC_CALIB_POINT_2;
    adc_conversion.gain = gain_pos_3v6;
    adc_conversion.offset = offset_pos_3v6;
    // Perform battery level detector measurement.
    status = _ADC_single_conversion(&adc_conversion, &mcu_voltage_12bits);
    if (status != MCAL_SUCCESS) goto errors;
    // Convert to mV.
    (*mcu_voltage_mv) = __LL_ADC_CALC_DATA_TO_VOLTAGE(LL_ADC_VIN_RANGE_3V6, mcu_voltage_12bits, LL_ADC_DS_DATA_WIDTH_12_BIT);
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t ADC_get_mcu_temperature(int16_t *mcu_temperature_degrees) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ADC_conversion_t adc_conversion;
    uint32_t mcu_temperature_12bits = 0;
    // Check parameter.
    if (mcu_temperature_degrees == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Reset result to default.
    (*mcu_temperature_degrees) = ADC_TMCU_DEGREES_DEFAULT;
    // Enable temperature sensor.
    LL_PWR_EnableTempSens();
    // Configure conversion.
    adc_conversion.channel = LL_ADC_CHANNEL_TEMPSENSOR;
    adc_conversion.range = LL_ADC_VIN_RANGE_1V2;
    adc_conversion.calibration_point = LL_ADC_CALIB_POINT_1;
    adc_conversion.gain = 0;
    adc_conversion.offset = 0;
    // Perform temperature sensor measurement.
    status = _ADC_single_conversion(&adc_conversion, &mcu_temperature_12bits);
    if (status != MCAL_SUCCESS) goto errors;
    // Convert to degrees.
    (*mcu_temperature_degrees) = __LL_ADC_CALC_TEMPERATURE(mcu_temperature_12bits, LL_ADC_DS_DATA_WIDTH_12_BIT);
errors:
    LL_PWR_DisableTempSens();
    return status;
}
