/*!*****************************************************************
 * \file    main.c
 * \brief   Main file.
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
#include "lptim.h"
#include "mcal.h"
#include "pwr.h"
#include "rcc.h"
#include "stdint.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_ep_api.h"
#include "sigfox_error.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"

/*** APP local macros ***/

#define APP_ERROR_STACK_DEPTH	10
#ifdef UL_PAYLOAD_SIZE
#define APP_UL_PAYLOAD_SIZE		UL_PAYLOAD_SIZE
#else
#define APP_UL_PAYLOAD_SIZE		9
#endif

/*** APP local structures ***/

// Applicative error codes.
typedef enum {
    APP_SUCCESS = 0,
	APP_ERROR_MCAL,
    APP_ERROR_SIGFOX_EP_LIBRARY,
    APP_ERROR_LAST
} APP_status_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned button_press : 1;
		unsigned sigfox_process : 1;
		unsigned sigfox_message_completion : 1;
	} field;
	uint8_t all;
} APP_flags_t;

/*******************************************************************/
typedef struct {
	volatile APP_flags_t flags;
	APP_status_t error_stack[APP_ERROR_STACK_DEPTH];
	uint8_t error_stack_index;
} APP_context_t;

/*** APP local global variables ***/

static APP_context_t app_ctx;

/*** APP local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) { _APP_stack_error(APP_ERROR_MCAL); } }

#ifdef ERROR_CODES
/*******************************************************************/
#define _check_sigfox_ep_api_status(void) { if (sigfox_ep_api_status != SIGFOX_EP_API_SUCCESS) _APP_stack_error(APP_ERROR_SIGFOX_EP_LIBRARY); }
#endif

/*******************************************************************/
static void _APP_button_press_callback(void) {
	// Set flag.
	app_ctx.flags.field.button_press = 1;
}

#ifdef ASYNCHRONOUS
/*******************************************************************/
static void _APP_sigfox_process_callback(void) {
	// Set flag.
	app_ctx.flags.field.sigfox_process = 1;
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
static void _APP_sigfox_message_completion_callback(void) {
	// Set flag.
	app_ctx.flags.field.sigfox_message_completion = 1;
}
#endif

/*******************************************************************/
static void _APP_stack_error(APP_status_t code) {
	// Store code.
	app_ctx.error_stack[app_ctx.error_stack_index] = code;
	app_ctx.error_stack_index = (app_ctx.error_stack_index + 1) % APP_ERROR_STACK_DEPTH;
}

/*******************************************************************/
static void _APP_init_context(void) {
	// Local variables.
	uint8_t idx = 0;
	// Clear all flags.
	app_ctx.flags.all = 0;
	// Init stack.
	for (idx=0 ; idx<APP_ERROR_STACK_DEPTH ; idx++) app_ctx.error_stack[idx] = APP_SUCCESS;
	app_ctx.error_stack_index = 0;
}

/*******************************************************************/
static void _APP_init_board(void) {
	// Local variables.
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Init power module and clock tree.
	PWR_init();
	RCC_init();
	// Switch to 16MHz internal clock.
	mcal_status = RCC_switch_to_hsi();
	_check_mcal_status();
	// Init GPIOs.
	GPIO_init();
	EXTI_init();
	// Init delay timer.
	mcal_status = LPTIM_init();
	_check_mcal_status();
	// Init components.
	mcal_status = BUTTON_init(&_APP_button_press_callback);
	_check_mcal_status();
}

/*******************************************************************/
static void _APP_open_sigfox_library(void) {
	// Local variables.
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif
	SIGFOX_EP_API_config_t lib_config;
	// Configure library.
	lib_config.rc = &SIGFOX_RC1;
#ifndef MESSAGE_COUNTER_ROLLOVER
	lib_config.message_counter_rollover = SIGFOX_MESSAGE_COUNTER_ROLLOVER_4096;
#endif
#ifdef ASYNCHRONOUS
	lib_config.process_cb = &_APP_sigfox_process_callback;
#endif
	// Open library.
#ifdef ERROR_CODES
	sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
	_check_sigfox_ep_api_status();
#else
	SIGFOX_EP_API_open(&lib_config);
#endif
}

/*******************************************************************/
static void _APP_send_sigfox_message(void) {
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef APPLICATION_MESSAGES
	SIGFOX_EP_API_application_message_t application_message;
#else
	SIGFOX_EP_API_control_message_t application_message;
#endif
#ifdef APPLICATION_MESSAGES
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	uint8_t idx = 0;
	sfx_u8 app_ul_payload[APP_UL_PAYLOAD_SIZE];
	// Build payload.
	for (idx=0 ; idx<APP_UL_PAYLOAD_SIZE ; idx++) app_ul_payload[idx] = (sfx_u8) idx;
#endif
#endif
	// Configure message.
#ifndef UL_BIT_RATE_BPS
	application_message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#endif
#ifndef TX_POWER_DBM_EIRP
	application_message.common_parameters.tx_power_dbm_eirp = 14;
#endif
#ifndef SINGLE_FRAME
	application_message.common_parameters.number_of_frames = 3;
#ifndef T_IFU_MS
	application_message.common_parameters.t_ifu_ms = 1000;
#endif
#endif
#ifdef PUBLIC_KEY_CAPABLE
	application_message.common_parameters.ep_key_type = SIGFOX_EP_KEY_PRIVATE;
#endif
#ifdef APPLICATION_MESSAGES
#ifdef UL_PAYLOAD_SIZE
#if (UL_PAYLOAD_SIZE == 0)
	application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
#else
	application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#endif
#else
	application_message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
#endif
#else
	application_message.type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE;
#endif
#ifdef ASYNCHRONOUS
	application_message.uplink_cplt_cb = SFX_NULL;
#ifdef BIDIRECTIONAL
	application_message.downlink_cplt_cb = SFX_NULL;
#endif
	application_message.message_cplt_cb = &_APP_sigfox_message_completion_callback;
#endif
#ifdef APPLICATION_MESSAGES
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	application_message.ul_payload = (sfx_u8*) app_ul_payload;
#endif
#ifndef UL_PAYLOAD_SIZE
	application_message.ul_payload_size_bytes = APP_UL_PAYLOAD_SIZE;
#endif
#ifdef BIDIRECTIONAL
	application_message.bidirectional_flag = SFX_FALSE;
#ifndef T_CONF_MS
	application_message.t_conf_ms = 2000;
#endif
#endif
#endif
	// Send message.
#ifdef ERROR_CODES
#ifdef APPLICATION_MESSAGES
	sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&application_message);
#else
	sigfox_ep_api_status = SIGFOX_EP_API_send_control_message(&application_message);
#endif
	_check_sigfox_ep_api_status();
#else
#ifdef APPLICATION_MESSAGES
	SIGFOX_EP_API_send_application_message(&application_message);
#else
	SIGFOX_EP_API_send_control_message(&application_message);
#endif
#endif
}

/*******************************************************************/
static void _APP_close_sigfox_library(void) {
	// Local variables.
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif
	// Close library.
#ifdef ERROR_CODES
	sigfox_ep_api_status = SIGFOX_EP_API_close();
	_check_sigfox_ep_api_status();
#else
	SIGFOX_EP_API_close();
#endif
}

#ifdef ASYNCHRONOUS
/*******************************************************************/
static uint8_t _APP_is_sigfox_library_running(void) {
	// Local variables.
	return (((SIGFOX_EP_API_get_state() == SIGFOX_EP_API_STATE_CLOSED) || (SIGFOX_EP_API_get_state() == SIGFOX_EP_API_STATE_READY)) ? 0 : 1);
}
#endif

/*** APP function ***/

/*******************************************************************/
int main(void) {
	// Local variables.
	MCAL_status_t mcal_status = MCAL_SUCCESS;
	// Init board and context.
	_APP_init_context();
	_APP_init_board();
	// Enable button interrupt.
	mcal_status = BUTTON_enable_irq();
	_check_mcal_status();
	// Main loop.
	while (1) {
		// Check if all events have been processed.
		if (app_ctx.flags.all == 0) {
#ifdef ASYNCHRONOUS
			// Enter sleep or stop mode depending on Sigfox library state.
			if (_APP_is_sigfox_library_running() == 0) {
				PWR_enter_stop_mode();
			}
			else {
				PWR_enter_sleep_mode();
			}
#else
			// Enter stop mode.
			PWR_enter_stop_mode();
#endif
		}
		// Check button press.
		if (app_ctx.flags.field.button_press != 0) {
			// Disable interrupt during processing.
			mcal_status = BUTTON_disable_irq();
			_check_mcal_status();
			// Clear flag.
			app_ctx.flags.field.button_press = 0;
			// Send message.
			_APP_open_sigfox_library();
			_APP_send_sigfox_message();
#ifndef ASYNCHRONOUS
			app_ctx.flags.field.sigfox_message_completion = 1;
#endif
		}
		// Check Sigfox completion flag.
		if (app_ctx.flags.field.sigfox_message_completion != 0) {
			// Clear flag.
			app_ctx.flags.field.sigfox_message_completion = 0;
			// Close library.
			_APP_close_sigfox_library();
			// Re-enable interrupt.
			mcal_status = BUTTON_enable_irq();
			_check_mcal_status();
		}
#ifdef ASYNCHRONOUS
		// Check Sigfox process flag.
		if (app_ctx.flags.field.sigfox_process != 0) {
			// Clear flag.
			app_ctx.flags.field.sigfox_process = 0;
			// Call process function.
			SIGFOX_EP_API_process();
		}
#endif
	}
	return 0;
}
