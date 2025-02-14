/*!*****************************************************************
 * \file    pwr.c
 * \brief   Common PWR driver based on LL driver.
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

#include "pwr.h"

#include "mcal.h"
#include "stdint.h"
#include "stddef.h"
#include "cmsis_gcc.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_cortex.h"
#include "stm32wl3x_ll_pwr.h"
#include "stm32wl3x_ll_rcc.h"
#include "stm32wl3x_ll_system.h"

/*** PWR local macros ***/

#define PWR_TIMEOUT_COUNT   10000000

/*** PWR local global variables ***/

static const uint32_t PWR_LL_SMPS_VOLTAGE[PWR_SMPS_VOLTAGE_LAST] = {
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V20,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V30,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V50,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V60,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V70,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V80,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_1V90,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_2V00,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_2V10,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_2V20,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_2V30,
    LL_PWR_SMPS_OUTPUT_VOLTAGE_2V40,
};

/*** PWR functions ***/

/*******************************************************************/
void PWR_init(void) {
    // Set SMPS BOM.
    LL_PWR_SetSMPSBOM(LL_PWR_SMPS_BOM3);
    // Use GPIO peripheral to control pull resistors.
    LL_PWR_DisablePUPDCfg();
    // Enable GPIO and RAM retention.
    LL_PWR_EnableRAMBankRet();
    LL_PWR_EnableGPIORET();
    LL_PWR_EnableDBGRET();
}

/*******************************************************************/
void PWR_enter_sleep_mode(void) {
    // Enter low power sleep mode.
    LL_LPM_EnableSleep();
    __WFI();
}

/*******************************************************************/
void PWR_enter_low_power_sleep_mode(void) {
    // Not available on STM32WL3X.
    PWR_enter_sleep_mode();
}

/*******************************************************************/
void PWR_enter_stop_mode(void) {
    // Note: both DEEPSTOP and SHUTDOWN modes reboots the CPU on wakeup, requiring a context saving which is not implemented in this example.
    // In a real application, the DEEPSTOP mode should be used to reduce the power consumption.
    PWR_enter_sleep_mode();
}

/*******************************************************************/
MCAL_status_t PWR_set_smps_voltage(PWR_smps_voltage_t smps_voltage) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    // Check voltage.
    if (smps_voltage >= PWR_SMPS_VOLTAGE_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Bypass mode.
    LL_PWR_SetSMPSPrechargeMode(LL_PWR_SMPS_PRECHARGE);
    // Wait for SMPS to be disabled.
    while (LL_PWR_IsSMPSReady() != 0) {
        loop_count++;
        if (loop_count > PWR_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Program output level.
    LL_PWR_SMPS_SetOutputVoltageLevel(PWR_LL_SMPS_VOLTAGE[smps_voltage]);
    // Disable bypass.
    LL_PWR_SetSMPSPrechargeMode(LL_PWR_NO_SMPS_PRECHARGE);
    // Wait for SMPS to be ready.
    while (LL_PWR_IsSMPSReady() != 0) {
        loop_count++;
        if (loop_count > PWR_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t PWR_get_smps_voltage(PWR_smps_voltage_t *smps_voltage) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t smps_level = LL_PWR_SMPS_GetOutputVoltageLevel();
    uint8_t level_found = 0;
    uint8_t idx = 0;
    // Check parameter.
    if (smps_voltage == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Convert to voltage.
    for (idx = 0; idx < PWR_SMPS_VOLTAGE_LAST ; idx++) {
        if (smps_level == PWR_LL_SMPS_VOLTAGE[idx]) {
            (*smps_voltage) = idx;
            level_found = 1;
            break;
        }
    }
    if (level_found == 0) {
        status = MCAL_ERROR;
        goto errors;
    }
errors:
    return status;
}
