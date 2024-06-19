/*!*****************************************************************
 * \file    sx126x_hw_api.c
 * \brief   Sigfox SX126X HW API implementation.
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

#include "board/sx126x_hw_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_error.h"

#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "nvic.h"
#include "stdint.h"
#include "stddef.h"
#include "spi.h"
#include "sx126x_mapping.h"

/*** SX126X HW API local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) EXIT_ERROR(SX126X_HW_API_ERROR); }

/*** SX126X HW API functions ***/

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_open(SX126X_HW_irq_cb_t callback) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Configure hardware interface.
	mcal_status = GPIO_configure(&SX126X_GPIO_NSS, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_write(&SX126X_GPIO_NSS, 1);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_ANT_SW, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_write(&SX126X_GPIO_ANT_SW, 1);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_NRESET, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_IRQ, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_BUSY, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_UP);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_LED_TX, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_LED_RX, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	// Init SPI peripheral.
	mcal_status = SPI_init();
	_check_mcal_status();
	// Init IRQ line.
	mcal_status = EXTI_configure(SX126X_GPIO_IRQ_EXTI_PORT, SX126X_GPIO_IRQ_EXTI_LINE, EXTI_TRIGGER_RISING, callback);
	_check_mcal_status();
	mcal_status = EXTI_enable_irq(SX126X_GPIO_IRQ_EXTI_LINE, NVIC_IRQ_PRIORITY_EXTI_RADIO);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_close(void) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Disable EXTI line.
	mcal_status = EXTI_de_configure(SX126X_GPIO_IRQ_EXTI_LINE);
	_check_mcal_status();
	// Release SPI peripheral.
	mcal_status = SPI_de_init();
	_check_mcal_status();
	// Put GPIOs in high impedance.
	mcal_status = GPIO_configure(&SX126X_GPIO_NSS, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_ANT_SW, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_NRESET, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_IRQ, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_BUSY, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_LED_TX, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&SX126X_GPIO_LED_RX, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_delayMs(unsigned short delay_ms) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Use LPTIM.
	mcal_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_chip_name(SX126X_HW_API_chip_name_t *chipset) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	// Check parameter.
	if (chipset == NULL) EXIT_ERROR(SX126X_HW_API_ERROR);
	// Set chipset.
	(*chipset) = SX126X_HW_API_CHIP_NAME_SX1261;
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_reg_mode(SX126X_HW_API_reg_mod_t *reg_mode) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	// Check parameter.
	if (reg_mode == NULL) EXIT_ERROR(SX126X_HW_API_ERROR);
	// Set regulator mode.
	(*reg_mode) = SX126X_HW_API_REG_MODE_DCDC;
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_xosc_cfg(SX126X_HW_API_xosc_cfg_t *xosc_cfg) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	// Check parameter.
	if (xosc_cfg == NULL) EXIT_ERROR(SX126X_HW_API_ERROR);
	// Set oscillator configuration.
	xosc_cfg -> tcxo_is_radio_controlled = 0;
	xosc_cfg -> supply_voltage = SX126X_HW_API_TCXO_CTRL_3_0V;
	xosc_cfg -> startup_time_in_tick = 300;
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_pa_pwr_cfg(SX126X_HW_API_pa_pwr_cfg_t *pa_pwr_cfg, sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	// Ignore unused parameters.
	MCAL_UNUSED(rf_freq_in_hz);
	MCAL_UNUSED(expected_output_pwr_in_dbm);
	// Check parameter.
	if (pa_pwr_cfg == NULL) EXIT_ERROR(SX126X_HW_API_ERROR);
	// Set PA configuration.
	pa_pwr_cfg -> power = 14;
	pa_pwr_cfg -> pa_config.hp_max = 0x00;
	pa_pwr_cfg -> pa_config.pa_duty_cycle = 0x07;
	pa_pwr_cfg -> pa_config.device_sel = 0x01;
	pa_pwr_cfg -> pa_config.pa_lut = 0x01;
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_tx_on(void) {
	// Local variables.
#ifdef ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn TX LED on.
	mcal_status = GPIO_write(&SX126X_GPIO_LED_TX, 1);
	_check_mcal_status();
errors:
    RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_tx_off(void) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn TX LED off.
	mcal_status = GPIO_write(&SX126X_GPIO_LED_TX, 0);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_rx_on(void) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn RX LED on.
	mcal_status = GPIO_write(&SX126X_GPIO_LED_RX, 1);
	_check_mcal_status();
errors:
	RETURN();
}
/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_rx_off(void) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn RX LED off.
	mcal_status = GPIO_write(&SX126X_GPIO_LED_RX, 0);
	_check_mcal_status();
errors:
	RETURN();
}

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
SX126X_HW_API_status_t SX126X_HW_API_get_latency(SX126X_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
	// Local variables.
#ifdef ERROR_CODES
	SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif
	// Check parameter.
	if (latency_ms == NULL) EXIT_ERROR(SX126X_HW_API_ERROR);
	// Update latency value.
	switch (latency_type) {
	case SX126X_HW_API_LATENCY_RESET:
		(*latency_ms) = SX126X_HAL_RESET_DELAY_MS;
		break;
	case SX126X_HW_API_LATENCY_WAKEUP:
		(*latency_ms) = SX126X_HAL_WAKEUP_DELAY_MS;
		break;
	default:
		EXIT_ERROR(SX126X_HW_API_ERROR);
		break;
	}
errors:
	RETURN();
}
#endif
