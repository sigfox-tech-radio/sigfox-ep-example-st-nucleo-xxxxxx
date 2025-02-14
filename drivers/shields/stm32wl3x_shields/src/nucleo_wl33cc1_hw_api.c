/*!*****************************************************************
 * \file    nucleo_wl3cc1_hw_api.c
 * \brief   Sigfox STM32WL3x HW interface.
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

#include "board/stm32wl3x_hw_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_error.h"
#include "sigfox_types.h"
#include "manuf/rf_api.h"
#include "stm32wl3x_ll_pwr.h"

#include "gpio.h"
#include "mcal.h"
#include "pwr.h"
#include "tim.h"

/*** STM32WL3X HW API local global macros ***/

#define STM32WL3X_HW_API_INIT_TX_DELAY_MS   50

#define STM32WL3X_HW_API_TX_POWER_DBM_MAX   14
#define STM32WL3X_HW_API_PA_DRIVE_MODE      PA_DRV_TX_HP

/*** STM32WL3X HW API local global variables ***/

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
// Latency values.
static sfx_u32 STM32WL3X_HW_API_LATENCY_MS[STM32WL3X_HW_API_LATENCY_LAST] = {
    STM32WL3X_HW_API_INIT_TX_DELAY_MS, // TX init.
    0, // TX de-init.
#ifdef SIGFOX_EP_BIDIRECTIONAL
    0, // RX init.
    0  // RX de-init.
#endif
};
#endif

/*** STM32WL3X HW API local structures ***/

/*******************************************************************/
typedef struct {
    PWR_smps_voltage_t previous_smps_voltage;
} STM32WL3X_HW_API_context_t;

/*** STM32WL3X HW API local global variables ***/

static STM32WL3X_HW_API_context_t stm32wl3x_hw_api_ctx;

/*** STM32WL3X HW API local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) SIGFOX_EXIT_ERROR(STM32WL3X_HW_API_ERROR); }

/*** STM32WL3X HW API functions ***/

/*******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_open(STM32WL3X_HW_API_config_t *hw_api_config) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Unused parameter.
    SIGFOX_UNUSED(hw_api_config);
    // Configure LEDs pin.
    mcal_status = GPIO_configure(GPIO_PIN_LED_BLUE, GPIO_MODE_DIGITAL_OUTPUT, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_OUTPUT_SPEED_LOW, GPIO_PULL_NONE, 0);
    _check_mcal_status();
    mcal_status = GPIO_write(GPIO_PIN_LED_BLUE, 1);
    _check_mcal_status();
#ifdef SIGFOX_EP_BIDIRECTIONAL
    mcal_status = GPIO_configure(GPIO_PIN_LED_GREEN, GPIO_MODE_DIGITAL_OUTPUT, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_OUTPUT_SPEED_LOW, GPIO_PULL_NONE, 0);
    _check_mcal_status();
    mcal_status = GPIO_write(GPIO_PIN_LED_GREEN, 1);
    _check_mcal_status();
#endif
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_close(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Release LEDs pin.
    mcal_status = GPIO_configure(GPIO_PIN_LED_BLUE, GPIO_MODE_ANALOG_INPUT, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_OUTPUT_SPEED_LOW, GPIO_PULL_NONE, 0);
    _check_mcal_status();
#ifdef SIGFOX_EP_BIDIRECTIONAL
    mcal_status = GPIO_configure(GPIO_PIN_LED_GREEN, GPIO_MODE_ANALOG_INPUT, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_OUTPUT_SPEED_LOW, GPIO_PULL_NONE, 0);
    _check_mcal_status();
#endif
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_init(STM32WL3X_radio_parameters_t *radio_parameters) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    PWR_smps_voltage_t smps_voltage = PWR_SMPS_VOLTAGE_1V4;
    // Save current SMPS settings.
    mcal_status = PWR_get_smps_voltage(&stm32wl3x_hw_api_ctx.previous_smps_voltage);
    _check_mcal_status();
    // Check radio mode.
    switch (radio_parameters->rf_mode) {
    case RF_API_MODE_TX:
        // Set SMPS according to PA drive mode.
        switch (STM32WL3X_HW_API_PA_DRIVE_MODE) {
        case PA_DRV_TX:
        case PA_DRV_TX_HP:
            smps_voltage = PWR_SMPS_VOLTAGE_1V4;
            break;
        case PA_DRV_TX_TX_HP:
            smps_voltage = PWR_SMPS_VOLTAGE_2V2;
            break;
        default:
            SIGFOX_EXIT_ERROR(STM32WL3X_HW_API_ERROR);
            goto errors;
        }
        mcal_status = PWR_set_smps_voltage(smps_voltage);
        _check_mcal_status();
        // This timer is only used for RSA compatibility (consecutive frames with small interframe are not properly demodulated).
        mcal_status = TIM_start(TIM_CHANNEL_CH4, STM32WL3X_HW_API_INIT_TX_DELAY_MS, TIM_WAITING_MODE_ACTIVE, SIGFOX_NULL);
        _check_mcal_status();
        mcal_status = TIM_wait_completion(TIM_CHANNEL_CH4);
        _check_mcal_status();
        mcal_status = TIM_stop(TIM_CHANNEL_CH4);
        _check_mcal_status();
        // Turn blue LED on.
        mcal_status = GPIO_write(GPIO_PIN_LED_BLUE, 0);
        _check_mcal_status();
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case RF_API_MODE_RX:
        // Turn green LED on.
        mcal_status = GPIO_write(GPIO_PIN_LED_GREEN, 0);
        _check_mcal_status();
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR(STM32WL3X_HW_API_ERROR);
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_de_init(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Restore previous SMPS configuration.
    mcal_status = PWR_set_smps_voltage(stm32wl3x_hw_api_ctx.previous_smps_voltage);
    _check_mcal_status();
    // Turn LEDs off.
    mcal_status = GPIO_write(GPIO_PIN_LED_BLUE, 1);
    _check_mcal_status();
#ifdef SIGFOX_EP_BIDIRECTIONAL
    mcal_status = GPIO_write(GPIO_PIN_LED_GREEN, 1);
    _check_mcal_status();
#endif
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *tx_power_dbm, MRSubG_PA_DRVMode *pa_drive_mode) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    // No gain to compensate on this shield.
    (*tx_power_dbm) = (expected_tx_power_dbm > STM32WL3X_HW_API_TX_POWER_DBM_MAX) ? STM32WL3X_HW_API_TX_POWER_DBM_MAX : expected_tx_power_dbm;
    (*pa_drive_mode) = STM32WL3X_HW_API_PA_DRIVE_MODE;
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_get_latency(STM32WL3X_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    // Update latency.
    (*latency_ms) = STM32WL3X_HW_API_LATENCY_MS[latency_type];
    SIGFOX_RETURN();
}
#endif
