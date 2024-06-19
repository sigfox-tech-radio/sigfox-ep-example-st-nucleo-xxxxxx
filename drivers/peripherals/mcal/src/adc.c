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

#include "lptim.h"
#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#ifdef STM32L0XX
#include "stm32l0xx_ll_adc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_pwr.h"
#endif

/*** ADC local macros ***/

#define ADC_INSTANCE				ADC1

#define ADC_REGULATOR_DELAY_MS		2
#define ADC_VREF_TS_DELAY_MS		2

#define ADC_VMCU_MV_DEFAULT			3300
#define ADC_TMCU_DEGREES_DEFAULT	25

#define ADC_TIMEOUT_COUNT			1000000

/*** ADC local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
static MCAL_status_t _ADC_single_conversion(uint32_t channel, uint16_t* adc_data_12bits) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	uint32_t loop_count = 0;
	// Clear flag.
	LL_ADC_ClearFlag_EOS(ADC_INSTANCE);
	// Configure channel.
	LL_ADC_REG_SetSequencerChannels(ADC_INSTANCE, channel);
	LL_ADC_REG_StartConversion(ADC_INSTANCE);
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
	(*adc_data_12bits) = LL_ADC_REG_ReadConversionData12(ADC_INSTANCE);
errors:
	return status;
}

/*** ADC functions ***/

/*******************************************************************/
MCAL_status_t ADC_init(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	ErrorStatus ll_status;
	LL_ADC_CommonInitTypeDef adc_common_init;
	LL_ADC_InitTypeDef adc_init;
	uint32_t loop_count = 0;
	// Enable peripheral clock.
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	// Enable ADC voltage regulator.
	LL_ADC_EnableInternalRegulator(ADC_INSTANCE);
	status = LPTIM_delay_milliseconds(ADC_REGULATOR_DELAY_MS, LPTIM_DELAY_MODE_ACTIVE);
	if (status != MCAL_SUCCESS) goto errors;
	// Configure common parameters.
	adc_common_init.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
	ll_status = LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC_INSTANCE), &adc_common_init);
	_check_ll_status();
	// Init ADC.
	adc_init.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
	adc_init.Resolution = LL_ADC_RESOLUTION_12B;
	adc_init.LowPowerMode = LL_ADC_LP_MODE_NONE;
	adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ll_status = LL_ADC_Init(ADC_INSTANCE, &adc_init);
	_check_ll_status();
	LL_ADC_REG_SetTriggerSource(ADC_INSTANCE, LL_ADC_REG_TRIG_SOFTWARE);
	LL_ADC_REG_SetContinuousMode(ADC_INSTANCE, LL_ADC_REG_CONV_SINGLE);
	LL_ADC_SetSamplingTimeCommonChannels(ADC_INSTANCE, LL_ADC_SAMPLINGTIME_160CYCLES_5);
	// Wait for internal reference to be ready.
	while (LL_PWR_IsActiveFlag_VREFINTRDY() == 0) {
		// Wait until VREFINT is ready or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = MCAL_ERROR;
			goto errors;
		}
	}
	// Calibration.
	LL_ADC_StartCalibration(ADC_INSTANCE);
	loop_count = 0;
	while ((LL_ADC_IsCalibrationOnGoing(ADC_INSTANCE) != 0) && (LL_ADC_IsActiveFlag_EOCAL(ADC_INSTANCE) == 0)) {
		// Wait until calibration is done or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = MCAL_ERROR;
			goto errors;
		}
	}
	// Enable ADC/
	LL_ADC_Enable(ADC_INSTANCE);
	loop_count = 0;
	while (LL_ADC_IsActiveFlag_ADRDY(ADC_INSTANCE) == 0) {
		// Wait for ADC to be ready (ADRDY='1') or timeout.
		loop_count++;
		if (loop_count > ADC_TIMEOUT_COUNT) {
			status = MCAL_ERROR;
			goto errors;
		}
	}
	// Enable internal reference and temperature sensor.
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC_INSTANCE), (LL_ADC_PATH_INTERNAL_VREFINT | LL_ADC_PATH_INTERNAL_TEMPSENSOR));
	status = LPTIM_delay_milliseconds(ADC_VREF_TS_DELAY_MS, LPTIM_DELAY_MODE_ACTIVE);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t ADC_de_init(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	ErrorStatus ll_status;
	// Release peripheral.
	ll_status = LL_ADC_DeInit(ADC_INSTANCE);
	_check_ll_status();
errors:
	// Disable ADC voltage regulator.
	LL_ADC_DisableInternalRegulator(ADC_INSTANCE);
	// Disable peripheral clock.
	LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_ADC1);
	return status;
}

/*******************************************************************/
MCAL_status_t ADC_get_mcu_voltage(uint16_t* mcu_voltage_mv) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	uint16_t mcu_voltage_12bits = 0;
	// Check parameter.
	if (mcu_voltage_mv == NULL) {
		status = MCAL_ERROR;
		goto errors;
	}
	// Reset result to default.
	(*mcu_voltage_mv) = ADC_VMCU_MV_DEFAULT;
	// Use VREFINT.
	status = _ADC_single_conversion(LL_ADC_CHANNEL_VREFINT, &mcu_voltage_12bits);
	if (status != MCAL_SUCCESS) goto errors;
	// Convert to mV.
	(*mcu_voltage_mv) = (uint16_t) __LL_ADC_CALC_VREFANALOG_VOLTAGE(mcu_voltage_12bits, LL_ADC_RESOLUTION_12B);
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t ADC_get_mcu_temperature(int16_t* mcu_temperature_degrees) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	uint16_t mcu_temperature_12bits = 0;
	// Check parameter.
	if (mcu_temperature_degrees == NULL) {
		status = MCAL_ERROR;
		goto errors;
	}
	// Reset result to default.
	(*mcu_temperature_degrees) = ADC_TMCU_DEGREES_DEFAULT;
	// Use temperature sensor.
	status = _ADC_single_conversion(LL_ADC_CHANNEL_TEMPSENSOR, &mcu_temperature_12bits);
	if (status != MCAL_SUCCESS) goto errors;
	// Convert to mV.
	(*mcu_temperature_degrees) = __LL_ADC_CALC_TEMPERATURE(ADC_VMCU_MV_DEFAULT, mcu_temperature_12bits, LL_ADC_RESOLUTION_12B);
errors:
	return status;
}
