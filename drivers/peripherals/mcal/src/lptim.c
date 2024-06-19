/*!*****************************************************************
 * \file    lptim.c
 * \brief   Common LPTIM driver based on LL driver.
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

#include "lptim.h"

#include "exti.h"
#include "mcal.h"
#include "nvic.h"
#include "pwr.h"
#include "stddef.h"
#include "stdint.h"
#ifdef STM32L0XX
#include "stm32l0xx.h"
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_lptim.h"
#include "stm32l0xx_ll_rcc.h"
#endif

/*** LPTIM local macros ***/

#define LPTIM_INSTANCE			LPTIM1

#define LPTIM_TIMEOUT_COUNT		1000000

#define LPTIM_ARR_MAX_VALUE		0xFFFF

#define LPTIM_DELAY_MS_MIN		2
#define LPTIM_DELAY_MS_MAX		((LPTIM_ARR_MAX_VALUE * 1000) / (lptim_ctx.clock_frequency_hz))

/*** LPTIM local structures ***/

/*******************************************************************/
typedef struct {
	uint32_t clock_frequency_hz;
	volatile uint8_t wake_up;
} LPTIM_context_t;

/*** LPTIM local global variables ***/

static LPTIM_context_t lptim_ctx;

/*** LPTIM local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
void __attribute__((optimize("-O0"))) LPTIM1_IRQHandler(void) {
	// Check flag.
	if (LL_LPTIM_IsActiveFlag_ARRM(LPTIM_INSTANCE) != 0) {
		// Set local flag.
		if (LL_LPTIM_IsEnabledIT_ARRM(LPTIM_INSTANCE) != 0) {
			lptim_ctx.wake_up = 1;
		}
		// Clear flag.
		LL_LPTIM_ClearFlag_ARRM(LPTIM_INSTANCE);
	}
	EXTI_clear_flag(EXTI_LINE_LPTIM1);
}

/*** LPTIM functions ***/

/*******************************************************************/
MCAL_status_t __attribute__((optimize("-O0"))) LPTIM_init(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	// Use LSE.
	lptim_ctx.clock_frequency_hz = (LSE_VALUE >> 3);
	// Enable LPTIM EXTI line.
	status = EXTI_configure(EXTI_PORT_NONE, EXTI_LINE_LPTIM1, EXTI_TRIGGER_RISING, NULL);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t __attribute__((optimize("-O0"))) LPTIM_delay_milliseconds(uint32_t delay_ms, LPTIM_delay_mode_t delay_mode) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	ErrorStatus ll_status = SUCCESS;
	LL_LPTIM_InitTypeDef lptim_init;
	uint32_t arr = 0;
	uint32_t loop_count = 0;
	// Check delay.
	if ((delay_ms < LPTIM_DELAY_MS_MIN) || (delay_ms > LPTIM_DELAY_MS_MAX)) {
		status = MCAL_ERROR;
		goto errors;
	}
	// Force APB clock to access registers.
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);
	// Enable peripheral clock.
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
	// Configure peripheral.
	lptim_init.ClockSource = LL_LPTIM_CLK_SOURCE_INTERNAL;
	lptim_init.Prescaler = LL_LPTIM_PRESCALER_DIV8;
	lptim_init.Waveform = LL_LPTIM_OUTPUT_WAVEFORM_PWM;
	lptim_init.Polarity = LL_LPTIM_OUTPUT_POLARITY_REGULAR;
	ll_status = LL_LPTIM_Init(LPTIM_INSTANCE, &lptim_init);
	_check_ll_status();
	// Enable interrupt.
	LL_LPTIM_EnableIT_ARRM(LPTIM_INSTANCE);
	// Reset flags.
	LL_LPTIM_ClearFlag_ARRM(LPTIM_INSTANCE);
	LL_LPTIM_ClearFlag_ARROK(LPTIM_INSTANCE);
	// Enable peripheral.
	LL_LPTIM_Enable(LPTIM_INSTANCE);
	// Compute ARR value.
	arr = LL_LPTIM_GetAutoReload(LPTIM_INSTANCE);
	arr &= 0xFFFF0000;
	arr |= ((((delay_ms - 1) * lptim_ctx.clock_frequency_hz) / (1000)) & 0x0000FFFF);
	// Write register.
	LL_LPTIM_SetAutoReload(LPTIM_INSTANCE, arr);
	// Wait for ARR write operation to complete.
	while (LL_LPTIM_IsActiveFlag_ARROK(LPTIM_INSTANCE) == 0) {
		loop_count++;
		if (loop_count > LPTIM_TIMEOUT_COUNT) {
			status = MCAL_ERROR;
			goto errors;
		}
	}
	// Select clock source.
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_LSE);
	// Clear wake-up flag.
	lptim_ctx.wake_up = 0;
	// Start timer.
	LL_LPTIM_StartCounter(LPTIM_INSTANCE, LL_LPTIM_OPERATING_MODE_ONESHOT);
	// Perform delay with the selected waiting mode.
	switch (delay_mode) {
	case LPTIM_DELAY_MODE_ACTIVE:
		// Active loop.
		while (LL_LPTIM_IsActiveFlag_ARRM(LPTIM_INSTANCE) == 0);
		break;
	case LPTIM_DELAY_MODE_SLEEP:
		// Enable interrupt.
		EXTI_enable_irq(EXTI_LINE_LPTIM1, NVIC_IRQ_PRIORITY_LPTIM);
		// Enter sleep mode.
		while (lptim_ctx.wake_up == 0) {
			PWR_enter_sleep_mode();
		}
		// Disable interrupt.
		EXTI_disable_irq(EXTI_LINE_LPTIM1);
		break;
	case LPTIM_DELAY_MODE_STOP:
		// Enable interrupt.
		EXTI_enable_irq(EXTI_LINE_LPTIM1, NVIC_IRQ_PRIORITY_LPTIM);
		// Enter stop mode.
		while (lptim_ctx.wake_up == 0) {
			PWR_enter_stop_mode();
		}
		// Disable interrupt.
		EXTI_disable_irq(EXTI_LINE_LPTIM1);
		break;
	default:
		status = MCAL_ERROR;
		goto errors;
	}
errors:
	// Reset peripheral.
	LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_LPTIM1);
	for (loop_count=0 ; loop_count<100 ; loop_count++);
	LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_LPTIM1);
	// Disable peripheral clock.
	LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_LPTIM1);
	// Force APB clock at the end of delay.
	LL_RCC_SetLPTIMClockSource(LL_RCC_LPTIM1_CLKSOURCE_PCLK1);
	return status;
}
