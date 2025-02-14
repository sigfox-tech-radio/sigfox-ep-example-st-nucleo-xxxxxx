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

#include "mcal.h"
#include "stdint.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_system.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT   1000000

/*** RCC local global variables ***/

static const uint32_t RCC_LL_MSI_RANGE[RCC_MSI_RANGE_LAST] = {
    LL_RCC_MSIRANGE_0,
    LL_RCC_MSIRANGE_1,
    LL_RCC_MSIRANGE_2,
    LL_RCC_MSIRANGE_3,
    LL_RCC_MSIRANGE_4,
    LL_RCC_MSIRANGE_5,
    LL_RCC_MSIRANGE_6,
};

/*** RCC functions ***/

/*******************************************************************/
MCAL_status_t __attribute__((optimize("-O0"))) RCC_init(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    // Reset backup domain.
    LL_RCC_ForceBackupDomainReset();
    for (loop_count = 0; loop_count < 100; loop_count++)
        ;
    LL_RCC_ReleaseBackupDomainReset();
    // Start low speed oscillator.
    LL_RCC_LSE_Enable();
    // Wait for LSE to be ready.
    loop_count = 0;
    while (LL_RCC_LSE_IsReady() == 0) {
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t RCC_switch_to_hsi(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    // Check current clock source.
    if (LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
        goto errors;
    }
    // Enable HSI.
    LL_RCC_HSI_Enable();
    // Wait for HSI to be stable.
    while (LL_RCC_HSI_IsReady() == 0) {
        // Wait for HSIRDYF='1' or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Set flash latency.
    LL_FLASH_SetLatency(1);
    // Switch SYSCLK.
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
    // Wait for clock switch.
    loop_count = 0;
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
        // Wait for SWS='01' or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t RCC_switch_to_msi(RCC_msi_range_t msi_range) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    // Check current clock source.
    if (LL_RCC_GetSysClkSource() == LL_RCC_SYS_CLKSOURCE_STATUS_MSI) {
        goto errors;
    }
    // Check parameter.
    if (msi_range >= RCC_MSI_RANGE_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Set frequency.
    LL_RCC_MSI_SetRange(RCC_LL_MSI_RANGE[msi_range]);
    // Enable MSI.
    LL_RCC_MSI_Enable();
    // Wait for MSI to be stable.
    while (((RCC->CR) & (0b1 << 9)) == 0) {
        // Wait for MSIRDYF='1' or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Switch SYSCLK.
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    // Wait for clock switch.
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI) {
        // Wait for SWS='00' or timeout.
        loop_count++;
        if (loop_count > RCC_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
    // Set flash latency.
    LL_FLASH_SetLatency(0);
errors:
    return status;
}
