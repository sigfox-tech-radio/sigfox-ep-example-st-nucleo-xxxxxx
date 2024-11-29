/*!*****************************************************************
 * \file    s2lp_hw_api.c
 * \brief   Sigfox S2LP HW API implementation.
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

#include "board/s2lp_hw_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
// Sigfox EP library.
#include "sigfox_error.h"
#include "sigfox_types.h"
#include "manuf/rf_api.h"

#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "nvic.h"
#include "mcal.h"
#include "s2lp_mapping.h"
#include "spi.h"

/*** S2LP HW API local macros ***/

#define S2LP_HW_API_OSCILLATOR_FREQUENCY_HZ     50000000
#define S2LP_HW_API_OPEN_DELAY_MS               100
#define S2LP_SHUTDOWN_DELAY_MS                  100

/*** S2LP HW API local global variables ***/

static const SPI_gpio_t S2LP_HW_API_SPI_GPIO = {
    &S2LP_GPIO_SPI_SCK,
    &S2LP_GPIO_SPI_MISO,
    &S2LP_GPIO_SPI_MOSI
};
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
// Latency values.
static sfx_u32 S2LP_HW_API_LATENCY_MS[S2LP_HW_API_LATENCY_LAST] = {
    S2LP_SHUTDOWN_DELAY_MS, // Exit shutdown.
    0, // Enter shutdown.
    0, // TX init TX.
    0, // TX de-init.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    0, // RX init.
    0  // RX de-init.
#endif
};
#endif

/*** S2LP HW API local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) SIGFOX_EXIT_ERROR(S2LP_HW_API_ERROR); }

/*** S2LP HW API functions ***/

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_open(S2LP_HW_API_config_t *hw_api_config) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Configure hardware interface.
    mcal_status = GPIO_configure(&S2LP_GPIO_SDN, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_write(&S2LP_GPIO_SDN, 1);
    _check_mcal_status();
    mcal_status = GPIO_configure(&S2LP_GPIO_IRQ, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    // Init SPI peripheral.
    mcal_status = SPI_init((SPI_gpio_t*) &S2LP_HW_API_SPI_GPIO);
    _check_mcal_status();
    mcal_status = GPIO_configure(&S2LP_GPIO_SPI_NSS, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_write(&S2LP_GPIO_SPI_NSS, 1);
    _check_mcal_status();
    // Init IRQ line.
    mcal_status = EXTI_configure(S2LP_GPIO_IRQ_EXTI_PORT, S2LP_GPIO_IRQ_EXTI_LINE, EXTI_TRIGGER_FALLING, (hw_api_config->gpio_irq_callback));
    _check_mcal_status();
    mcal_status = EXTI_enable_irq(S2LP_GPIO_IRQ_EXTI_LINE, NVIC_IRQ_PRIORITY_EXTI_RADIO);
    _check_mcal_status();
    // Init delay to ensure SDN is kept high during a minimum time.
    mcal_status = LPTIM_delay_milliseconds(S2LP_HW_API_OPEN_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_close(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Disable EXTI line.
    mcal_status = EXTI_de_configure(S2LP_GPIO_IRQ_EXTI_LINE);
    _check_mcal_status();
    // Release SPI peripheral.
    mcal_status = SPI_de_init((SPI_gpio_t*) &S2LP_HW_API_SPI_GPIO);
    _check_mcal_status();
    // Put GPIOs in high impedance.
    mcal_status = GPIO_configure(&S2LP_GPIO_SPI_NSS, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&S2LP_GPIO_SDN, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&S2LP_GPIO_IRQ, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_init(S2LP_radio_parameters_t *radio_parameters) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    // Ignore unused parameters.
    MCAL_UNUSED(radio_parameters);
    // Nothing to do on this shield.
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_de_init(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    // Nothing to do on this shield.
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_enter_shutdown(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Enter shutdown.
    mcal_status = GPIO_write(&S2LP_GPIO_SDN, 1);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_exit_shutdown(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Exit shutdown.
    mcal_status = GPIO_write(&S2LP_GPIO_SDN, 0);
    _check_mcal_status();
    // Startup delay.
    mcal_status = LPTIM_delay_milliseconds(S2LP_SHUTDOWN_DELAY_MS, LPTIM_DELAY_MODE_SLEEP);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_oscillator(S2LP_HW_API_oscillator_type_t *xo_type, sfx_u32 *xo_frequency_hz) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    // Update parameters.
    (*xo_type) = S2LP_HW_API_OSCILLATOR_TYPE_QUARTZ;
    (*xo_frequency_hz) = S2LP_HW_API_OSCILLATOR_FREQUENCY_HZ;
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_gpio(S2LP_HW_API_signal_t signal, S2LP_HW_API_gpio_t *s2lp_gpio) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    // Check signal.
    switch (signal) {
    case S2LP_HW_API_SIGNAL_NIRQ:
        // IRQ on GPIO3.
        (*s2lp_gpio) = S2LP_HW_API_GPIO_3;
        break;
    default:
#ifdef SIGFOX_EP_ERROR_CODES
        status = S2LP_HW_API_ERROR;
#endif
        break;
    }
    SIGFOX_RETURN();
}

/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *s2lp_tx_power_dbm) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    // No gain to compensate on this shield.
    (*s2lp_tx_power_dbm) = expected_tx_power_dbm;
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
S2LP_HW_API_status_t S2LP_HW_API_get_latency(S2LP_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    S2LP_HW_API_status_t status = S2LP_HW_API_SUCCESS;
#endif
    // Update latency.
    (*latency_ms) = S2LP_HW_API_LATENCY_MS[latency_type];
    SIGFOX_RETURN();
}
#endif
