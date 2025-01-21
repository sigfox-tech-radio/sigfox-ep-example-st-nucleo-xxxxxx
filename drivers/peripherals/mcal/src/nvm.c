/*!*****************************************************************
 * \file    nvm.c
 * \brief   Common NVM driver based on LL driver.
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

#include "nvm.h"

#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#ifdef STM32L0XX
#include "stm32l0xx_hal_flash.h"
#include "stm32l0xx_ll_bus.h"
#endif

/*** NVM local macros ***/

#define NVM_START_ADDRESS   0x08080000
#define NVM_TIMEOUT_COUNT   1000000

/*** NVM local functions ***/

/*******************************************************************/
#define _check_hal_status(void) { if (hal_status != HAL_OK) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
static MCAL_status_t _NVM_check_busy(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t loop_count = 0;
    // Check no write/erase operation is running.
    while ((__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) != 0) {
        // Wait till BSY='1' or timeout.
        loop_count++;
        if (loop_count > NVM_TIMEOUT_COUNT) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
errors:
    return status;
}

/*******************************************************************/
static MCAL_status_t __attribute__((optimize("-O0"))) _NVM_unlock(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    HAL_StatusTypeDef hal_status = HAL_OK;
    // Check memory is ready.
    status = _NVM_check_busy();
    if (status != MCAL_SUCCESS) {
        goto errors;
    }
    // Unlock sequence.
    hal_status = HAL_FLASH_Unlock();
    _check_hal_status();
errors:
    return status;
}

/*******************************************************************/
static MCAL_status_t __attribute__((optimize("-O0"))) _NVM_lock(void) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    HAL_StatusTypeDef hal_status = HAL_OK;
    // Check memory is ready.
    status = _NVM_check_busy();
    if (status != MCAL_SUCCESS) {
        goto errors;
    }
    // Lock sequence.
    hal_status = HAL_FLASH_Lock();
    _check_hal_status();
errors:
    return status;
}

/*** NVM functions ***/

/*******************************************************************/
MCAL_status_t __attribute__((optimize("-O0"))) NVM_read_byte(NVM_address_t address, uint8_t *data) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t absolute_address = (NVM_START_ADDRESS + address);
    // Check parameters.
    if ((address >= NVM_SIZE_BYTES) || (data == NULL)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Enable peripheral.
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_MIF);
    // Check there is no pending operation.
    status = _NVM_check_busy();
    if (status != MCAL_SUCCESS) {
        goto errors;
    }
    // Read data.
    (*data) = *((uint8_t *) (absolute_address));
errors:
    // Disable peripheral.
    LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_MIF);
    return status;
}

/*******************************************************************/
MCAL_status_t __attribute__((optimize("-O0"))) NVM_write_byte(NVM_address_t address, uint8_t data) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t absolute_address = (NVM_START_ADDRESS + address);
    // Check parameters.
    if (address >= NVM_SIZE_BYTES) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Enable peripheral.
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_MIF);
    // Unlock memory.
    status = _NVM_unlock();
    if (status != MCAL_SUCCESS) {
        goto errors;
    }
    // Write data.
    (*((uint8_t *) (absolute_address))) = data;
    // Wait the end of operation.
    status = _NVM_check_busy();
    if (status != MCAL_SUCCESS) {
        goto errors;
    }
    // Lock memory.
    status = _NVM_lock();
    if (status != MCAL_SUCCESS) {
        goto errors;
    }
    // Disable peripheral.
    LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_MIF);
    return status;
errors:
    // Lock memory.
    _NVM_lock();
    // Disable peripheral.
    LL_AHB1_GRP1_DisableClock(LL_AHB1_GRP1_PERIPH_MIF);
    return status;
}
