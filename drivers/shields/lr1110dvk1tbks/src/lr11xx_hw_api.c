/*!*****************************************************************
 * \file    lr11xx_hw_api.c
 * \brief   Sigfox LR11XX HW API implementation.
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
 
#include "board/lr11xx_hw_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
// Sigfox EP library.
#include "sigfox_error.h"
#include "sigfox_types.h"

#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "lr11xx_mapping.h"
#include "nvic.h"
#include "stdint.h"
#include "stddef.h"
#include "spi.h"

/*** LR11XX HW API local macros ***/

#define LR11XX_HW_API_RF_FREQ_MIN	150000000
#define LR11XX_HW_API_RF_FREQ_MAX	960000000

#define LR11XX_HW_API_RF_POWER_MIN	-17
#define LR11XX_HW_API_RF_POWER_MAX	22

/*** LR11XX HW API local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) EXIT_ERROR(LR11XX_HW_API_ERROR); }

/*** LR11XX HW API functions ***/

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_open(LR11XX_HW_irq_cb_t callback) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Configure hardware interface.
	mcal_status = GPIO_configure(&LR11XX_GPIO_NSS, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_write(&LR11XX_GPIO_NSS, 1);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_NRESET, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_IRQ, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_BUSY, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_UP);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_SCAN, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_LED_TX, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_LED_RX, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	// Init SPI peripheral.
	mcal_status = SPI_init();
	_check_mcal_status();
	// Init IRQ line.
	mcal_status = EXTI_configure(LR11XX_GPIO_IRQ_EXTI_PORT, LR11XX_GPIO_IRQ_EXTI_LINE, EXTI_TRIGGER_RISING, callback);
	_check_mcal_status();
	mcal_status = EXTI_enable_irq(LR11XX_GPIO_IRQ_EXTI_LINE, NVIC_IRQ_PRIORITY_EXTI_RADIO);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_close(void) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Disable EXTI line.
	mcal_status = EXTI_de_configure(LR11XX_GPIO_IRQ_EXTI_LINE);
	_check_mcal_status();
	// Release SPI peripheral.
	mcal_status = SPI_de_init();
	_check_mcal_status();
	// Put GPIOs in high impedance.
	mcal_status = GPIO_configure(&LR11XX_GPIO_NSS, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_SCAN, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_NRESET, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_IRQ, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_BUSY, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_LED_TX, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
	mcal_status = GPIO_configure(&LR11XX_GPIO_LED_RX, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_delayMs(unsigned short delay_ms) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Use LPTIM.
	mcal_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_fem_mask(LR11XX_HW_API_FEM_t fem, sfx_u8 *rfsw_dio_mask) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	switch (fem) {
	case LR11XX_HW_API_FEM_PIN_USED :
		(*rfsw_dio_mask) = LR11XX_HW_API_RFSW0_DIO5 | LR11XX_HW_API_RFSW1_DIO6 | LR11XX_HW_API_RFSW2_DIO7 | LR11XX_HW_API_RFSW3_DIO8;
		break;
	case LR11XX_HW_API_FEM_STBY :
		(*rfsw_dio_mask) = 0x00;
		break;
	case LR11XX_HW_API_FEM_RX :
		(*rfsw_dio_mask) = LR11XX_HW_API_RFSW0_DIO5;
		break;
	case LR11XX_HW_API_FEM_TX :
		(*rfsw_dio_mask) = LR11XX_HW_API_RFSW0_DIO5 | LR11XX_HW_API_RFSW1_DIO6;
		break;
	case LR11XX_HW_API_FEM_TXHP:
		(*rfsw_dio_mask) = LR11XX_HW_API_RFSW1_DIO6;
		break;
	case LR11XX_HW_API_FEM_WIFI:
		(*rfsw_dio_mask) = LR11XX_HW_API_RFSW3_DIO8;
		break;
	case LR11XX_HW_API_FEM_GNSS:
		(*rfsw_dio_mask) = LR11XX_HW_API_RFSW2_DIO7;
		break;
	default :
		(*rfsw_dio_mask) = 0x00;
		break;
	}
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_xosc_cfg(LR11XX_HW_API_xosc_cfg_t *xosc_cfg) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	// Update oscillator configuration.
	xosc_cfg -> has_tcxo = 0x01;
	xosc_cfg -> tcxo_supply_voltage = LR11XX_HW_API_TCXO_CTRL_1_8V;
	xosc_cfg -> startup_time_in_tick = 164;
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_pa_pwr_cfg(LR11XX_HW_API_pa_pwr_cfg_t *pa_pwr_cfg, sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm){
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	// Check frequency.
	if ((rf_freq_in_hz < LR11XX_HW_API_RF_FREQ_MIN) || (rf_freq_in_hz > LR11XX_HW_API_RF_FREQ_MAX)) {
		EXIT_ERROR(LR11XX_HW_API_ERROR);
	}
	// Check power.
	if ((expected_output_pwr_in_dbm < LR11XX_HW_API_RF_POWER_MIN) || (expected_output_pwr_in_dbm > LR11XX_HW_API_RF_POWER_MAX)) {
		EXIT_ERROR(LR11XX_HW_API_ERROR);
	}
	// Update PA configuration.
	pa_pwr_cfg -> power = expected_output_pwr_in_dbm;
	pa_pwr_cfg -> pa_config.pa_duty_cycle = (expected_output_pwr_in_dbm == 15) ? 0x07 : 0x04;
	// Check expected power.
	if (expected_output_pwr_in_dbm < 16) {
		pa_pwr_cfg -> pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP;
		pa_pwr_cfg -> pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG;
		pa_pwr_cfg -> pa_config.pa_duty_cycle = 0x00;
	}
	else {
		pa_pwr_cfg -> pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP;
		pa_pwr_cfg -> pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT;
		pa_pwr_cfg -> pa_config.pa_duty_cycle = 0x07;
	}
errors:
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_tx_on(void) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn TX LED on.
	mcal_status = GPIO_write(&LR11XX_GPIO_LED_TX, 1);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_tx_off(void) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn TX LED off.
	mcal_status = GPIO_write(&LR11XX_GPIO_LED_TX, 0);
	_check_mcal_status();
errors:
	RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_rx_on(void) {
	// Local variables.
#ifdef ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn RX LED on.
	mcal_status = GPIO_write(&LR11XX_GPIO_LED_RX, 1);
	_check_mcal_status();
errors:
    RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_rx_off(void) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Turn RX LED off.
	mcal_status = GPIO_write(&LR11XX_GPIO_LED_RX, 0);
	_check_mcal_status();
errors:
	RETURN();
}

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_latency(LR11XX_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
	// Local variables.
#ifdef ERROR_CODES
	LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
	// Check parameter.
	if (latency_ms == NULL) EXIT_ERROR(LR11XX_HW_API_ERROR);
	// Update latency value.
	switch (latency_type) {
	case LR11XX_HW_API_LATENCY_RESET:
		(*latency_ms) = LR11XX_HAL_RESET_DELAY_MS;
		break;
	case LR11XX_HW_API_LATENCY_WAKEUP:
		(*latency_ms) = LR11XX_HAL_WAKEUP_DELAY_MS;
		break;
	default:
		EXIT_ERROR(LR11XX_HW_API_ERROR);
		break;
	}
errors:
	RETURN();
}
#endif
