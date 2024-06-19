/*!*****************************************************************
 * \file    button.c
 * \brief   Common button driver based on MCAL.
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

#include "button.h"

#include "exti.h"
#include "gpio.h"
#include "nvic.h"
#include "stddef.h"
#include "stdint.h"

/*** BUTTON local macros ***/

#define BUTTON_EXTI_PORT	EXTI_PORT_C
#define BUTTON_EXTI_LINE	EXTI_LINE_GPIO_13

/*** BUTTON local global variables ***/

const GPIO_pin_t BUTTON_GPIO = (GPIO_pin_t) {GPIO_PORT_C, 13, 0};

/*** BUTTON functions ***/

/*******************************************************************/
MCAL_status_t BUTTON_init(BUTTON_press_irq_cb_t irq_callback) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	// Init GPIO.
	status = GPIO_configure(&BUTTON_GPIO, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
	// Init interrupt.
	status = EXTI_configure(BUTTON_EXTI_PORT, BUTTON_EXTI_LINE, EXTI_TRIGGER_FALLING, irq_callback);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t BUTTON_de_init(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	// Disable interrupt.
	status = EXTI_de_configure(BUTTON_EXTI_LINE);
	if (status != MCAL_SUCCESS) goto errors;
	// Put GPIO in high impedance.
	status = GPIO_configure(&BUTTON_GPIO, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t BUTTON_enable_irq(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	// Enable interrupt.
	status = EXTI_enable_irq(BUTTON_EXTI_LINE, NVIC_IRQ_PRIORITY_EXTI_BUTTON);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t BUTTON_disable_irq(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	// Disable interrupt.
	status = EXTI_disable_irq(BUTTON_EXTI_LINE);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	return status;
}
