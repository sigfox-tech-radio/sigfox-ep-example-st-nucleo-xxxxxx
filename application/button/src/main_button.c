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

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif /* SIGFOX_EP_DISABLE_FLAGS_FILE */
#include "sigfox_ep_api.h"
#include "sigfox_error.h"
#include "sigfox_rc.h"
#include "sigfox_types.h"

/*** APP local macros ***/

#define APP_ERROR_STACK_DEPTH   10
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#define APP_UL_PAYLOAD_SIZE     SIGFOX_EP_UL_PAYLOAD_SIZE
#else
#define APP_UL_PAYLOAD_SIZE     9
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */

/*** APP local structures ***/

#define DEFAULT_ANTENNA_GAIN_DBI   2

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
        unsigned button_press :1;
        unsigned sigfox_process :1;
        unsigned sigfox_message_completion :1;
    } field;
    uint8_t all;
} APP_flags_t;

/*******************************************************************/
typedef struct {
    volatile APP_flags_t flags;
    APP_status_t error_stack[APP_ERROR_STACK_DEPTH];
    uint8_t error_stack_index;
    const SIGFOX_rc_t *rc;
    SIGFOX_ul_bit_rate_t default_sigfox_ul_bit_rate;
} APP_context_t;

/*** APP local global variables ***/

static APP_context_t app_ctx;

/*** APP local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) { _APP_stack_error(APP_ERROR_MCAL); } }

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
#define _check_sigfox_ep_api_status(void) { if (sigfox_ep_api_status != SIGFOX_EP_API_SUCCESS) _APP_stack_error(APP_ERROR_SIGFOX_EP_LIBRARY); }
#endif /* SIGFOX_EP_ERROR_CODES */

/*******************************************************************/
static void _APP_button_press_callback(void) {
    // Set flag.
    app_ctx.flags.field.button_press = 1;
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
static void _APP_sigfox_process_callback(void) {
    // Set flag.
    app_ctx.flags.field.sigfox_process = 1;
}
#endif /* SIGFOX_EP_ASYNCHRONOUS */

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
static void _APP_sigfox_message_completion_callback(void) {
    // Set flag.
    app_ctx.flags.field.sigfox_message_completion = 1;
}
#endif /* SIGFOX_EP_ASYNCHRONOUS */

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
    for (idx = 0; idx < APP_ERROR_STACK_DEPTH; idx++) {
        app_ctx.error_stack[idx] = APP_SUCCESS;
    }
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
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    SIGFOX_EP_API_config_t lib_config;
    // Configure library.
#if defined(SIGFOX_EP_RC1_ZONE)
    app_ctx.rc = &SIGFOX_RC1;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#elif defined(SIGFOX_EP_RC2_ZONE)
    app_ctx.rc = &SIGFOX_RC2;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
#elif defined(SIGFOX_EP_RC3_LBT_ZONE)
    app_ctx.rc = &SIGFOX_RC3_LBT;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#elif defined(SIGFOX_EP_RC3_LDC_ZONE)
    app_ctx.rc = &SIGFOX_RC3_LDC;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#elif defined(SIGFOX_EP_RC4_ZONE)
    app_ctx.rc = &SIGFOX_RC4;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_600BPS;
#elif defined(SIGFOX_EP_RC5_ZONE)
    app_ctx.rc = &SIGFOX_RC5;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#elif defined(SIGFOX_EP_RC6_ZONE)
    app_ctx.rc = &SIGFOX_RC6;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#elif defined(SIGFOX_EP_RC7_ZONE)
    app_ctx.rc = &SIGFOX_RC7;
    app_ctx.default_sigfox_ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#endif /* SIGFOX_EP_RCx */
    lib_config.rc = app_ctx.rc;
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    lib_config.message_counter_rollover = SIGFOX_MESSAGE_COUNTER_ROLLOVER_4096;
#endif /* SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    lib_config.process_cb = &_APP_sigfox_process_callback;
#endif /* SIGFOX_EP_ASYNCHRONOUS */
    // Open library.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_api_status = SIGFOX_EP_API_open(&lib_config);
    _check_sigfox_ep_api_status();
#else
    SIGFOX_EP_API_open(&lib_config);
#endif /* SIGFOX_EP_ERROR_CODES */
}

/*******************************************************************/
static void _APP_send_sigfox_message(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    SIGFOX_EP_API_application_message_t message;
#else
    SIGFOX_EP_API_control_message_t message;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    uint8_t idx = 0;
    sfx_u8 app_ul_payload[APP_UL_PAYLOAD_SIZE];
    // Build payload.
    for (idx = 0; idx < APP_UL_PAYLOAD_SIZE; idx++) {
        app_ul_payload[idx] = (sfx_u8) idx;
    }
#endif /* !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0) */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
    // Configure message.
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    message.common_parameters.ul_bit_rate = SIGFOX_UL_BIT_RATE_100BPS;
#endif /* SIGFOX_EP_UL_BIT_RATE_BPS */
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    message.common_parameters.tx_power_dbm_eirp = app_ctx.rc->tx_power_dbm_eirp_max - DEFAULT_ANTENNA_GAIN_DBI;
#endif /* SIGFOX_EP_TX_POWER_DBM_EIRP */
#ifndef SIGFOX_EP_SINGLE_FRAME
    message.common_parameters.number_of_frames = 3;
#ifndef SIGFOX_EP_T_IFU_MS
    message.common_parameters.t_ifu_ms = 1000;
#endif /* SIGFOX_EP_T_IFU_MS */
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    message.common_parameters.ep_key_type = SIGFOX_EP_KEY_PRIVATE;
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
    message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
#else
    message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY;
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#else
    message.type = SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY;
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#else
    message.type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE;
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    message.uplink_cplt_cb = SIGFOX_NULL;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    message.downlink_cplt_cb = SIGFOX_NULL;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    message.message_cplt_cb = &_APP_sigfox_message_completion_callback;
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    message.ul_payload = (sfx_u8 *) app_ul_payload;
#endif /* !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0) */
#ifndef SIGFOX_EP_UL_PAYLOAD_SIZE
    message.ul_payload_size_bytes = APP_UL_PAYLOAD_SIZE;
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    message.bidirectional_flag = SIGFOX_FALSE;
#ifndef SIGFOX_EP_T_CONF_MS
    message.t_conf_ms = 2000;
#endif /* SIGFOX_EP_T_CONF_MS */
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
    // Send message.
#ifdef SIGFOX_EP_ERROR_CODES
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    sigfox_ep_api_status = SIGFOX_EP_API_send_application_message(&message);
#else
    sigfox_ep_api_status = SIGFOX_EP_API_send_control_message(&message);
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
    _check_sigfox_ep_api_status();
#else
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    SIGFOX_EP_API_send_application_message(&message);
#else
    SIGFOX_EP_API_send_control_message(&message);
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#endif /* SIGFOX_EP_ERROR_CODES */
}

/*******************************************************************/
static void _APP_close_sigfox_library(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t sigfox_ep_api_status = SIGFOX_EP_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    // Close library.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_api_status = SIGFOX_EP_API_close();
    _check_sigfox_ep_api_status();
#else
    SIGFOX_EP_API_close();
#endif /* SIGFOX_EP_ERROR_CODES */
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
static uint8_t _APP_is_sigfox_library_running(void) {
    // Local variables.
    return (((SIGFOX_EP_API_get_state() == SIGFOX_EP_API_STATE_CLOSED) || (SIGFOX_EP_API_get_state() == SIGFOX_EP_API_STATE_READY)) ? 0 : 1);
}
#endif /* SIGFOX_EP_ASYNCHRONOUS */

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
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Enter sleep or stop mode depending on Sigfox library state.
            if (_APP_is_sigfox_library_running() == 0) {
                PWR_enter_stop_mode();
            } else {
                PWR_enter_sleep_mode();
            }
#else
            // Enter stop mode.
            PWR_enter_stop_mode();
#endif /* SIGFOX_EP_ASYNCHRONOUS */
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
#ifndef SIGFOX_EP_ASYNCHRONOUS
            app_ctx.flags.field.sigfox_message_completion = 1;
#endif /* SIGFOX_EP_ASYNCHRONOUS */
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
#ifdef SIGFOX_EP_ASYNCHRONOUS
        // Check Sigfox process flag.
        if (app_ctx.flags.field.sigfox_process != 0) {
            // Clear flag.
            app_ctx.flags.field.sigfox_process = 0;
            // Call process function.
            SIGFOX_EP_API_process();
        }
#endif /* SIGFOX_EP_ASYNCHRONOUS */
    }
    return 0;
}
