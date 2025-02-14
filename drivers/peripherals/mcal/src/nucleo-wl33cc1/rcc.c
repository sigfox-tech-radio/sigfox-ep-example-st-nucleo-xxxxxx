/*!*****************************************************************
 * \file    rcc.c
 * \brief   Common RCC driver based on LL driver.
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

#include "rcc.h"

#include "stm32wl3x_ll_gpio.h"
#include "stm32wl3x_ll_rcc.h"
#include "stm32wl3x_ll_system.h"
#include "system_stm32wl3x.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT   1000000

/*** RCC local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*** RCC functions ***/

/*******************************************************************/
MCAL_status_t RCC_init(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    // Set SMPS prescaler.
    LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_4);
    // Set HSE settings.
    LL_RCC_HSE_SetCapacitorTuning(28);
    LL_RCC_HSE_SetStartupCurrent(0);
    LL_RCC_HSE_SetAmplitudeThreshold(0);
    LL_RCC_HSE_SetCurrentControl(40);
    // Start HSE.
    LL_RCC_HSE_Enable();
    // Wait for external oscillator to be ready.
    while (LL_RCC_HSE_IsReady() == 0) {
        // Manage timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Set prescaler to get 16MHz clock on peripheral side.
    LL_RCC_SetDirectHSEPrescaler(LL_RCC_DIRECT_HSE_DIV_3);
    // Increase flash latency.
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    // Use direct HSE.
    LL_RCC_DIRECT_HSE_Enable();
    // Wait for configuration to be done.
    loop_count = 0;
    while (LL_RCC_Get_DIRECT_HSESEL_Status() == 0) {
        // Manage timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
errors:
    SystemCoreClockUpdate();
    return status;
}

/*******************************************************************/
MCAL_status_t RCC_switch_to_hsi(void) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    return status;
}

/*******************************************************************/
MCAL_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(msi_range);
    return status;
}
