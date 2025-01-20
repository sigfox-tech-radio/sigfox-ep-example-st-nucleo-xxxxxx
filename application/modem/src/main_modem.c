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

#include "cli.h"
#include "gpio.h"
#include "mcal.h"
#include "rcc.h"
#include "pwr.h"
#include "lptim.h"
#include "exti.h"

/*** APP local macros ***/

#define APP_ERROR_STACK_DEPTH   10

/*** APP local structures ***/

// Applicative error codes.
typedef enum {
    APP_SUCCESS = 0,
    APP_ERROR_MCAL,
    APP_ERROR_CLI,
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
} APP_context_t;

/*** APP local global variables ***/

static APP_context_t app_ctx;

/*** APP local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) { _APP_stack_error(APP_ERROR_MCAL); } }

/*******************************************************************/
#define _check_cli_status(void) { if (cli_status != CLI_SUCCESS) { _APP_stack_error(APP_ERROR_CLI); } }

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
}

/*** APP function ***/

/*******************************************************************/
int main(void) {
    // Local variables.
    CLI_status_t cli_status = CLI_SUCCESS;
    // Init board and context.
    _APP_init_context();
    _APP_init_board();
    // Init CLI.
    cli_status = CLI_init();
    _check_cli_status();
    // Run CLI.
    while (1) {
        cli_status = CLI_process();
        _check_cli_status();
    }
    return 0;
}