/*!*****************************************************************
 * \file    sx126x_hal.c
 * \brief   SX126X library low level functions.
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

#include "sx126x_hal.h"

#include "gpio.h"
#include "lptim.h"
#include "mcal.h"
#include "spi.h"
#include "stdint.h"
#include "stddef.h"
#include "sx126x_mapping.h"

/*** SX126X HAL local macros ***/

#define SX126X_HAL_TIMEOUT_COUNT    1000000

/*** SX126X HAL local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) { status = SX126X_HAL_STATUS_ERROR; goto errors; } }

/*******************************************************************/
sx126x_hal_status_t _sx126x_hal_wait_busy(void) {
    // Local variables.
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    uint8_t gpio_busy = 0;
    uint32_t loop_count = 0;
    // Wait for chip to be ready.
    do {
        // Read pin.
        mcal_status = GPIO_read(&SX126X_MAPPING_gpios.busy, &gpio_busy);
        _check_mcal_status();
        // Manage timeout.
        loop_count++;
        if (loop_count > SX126X_HAL_TIMEOUT_COUNT) {
            status = SX126X_HAL_STATUS_ERROR;
            goto errors;
        }
    } while (gpio_busy != 0);
errors:
    return status;
}

/*** SX126X HAL functions ***/

/*******************************************************************/
sx126x_hal_status_t sx126x_hal_reset(const void *context) {
    // Local variables.
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Ignore unused parameters.
    MCAL_UNUSED(context);
    // Perform reset.
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.reset, 0);
    _check_mcal_status();
    mcal_status = LPTIM_delay_milliseconds(SX126X_HAL_RESET_DELAY_MS, LPTIM_DELAY_MODE_ACTIVE);
    _check_mcal_status();
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.reset, 1);
    _check_mcal_status();
errors:
    return status;
}

/*******************************************************************/
sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
    // Local variables.
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Ignore unused parameters.
    MCAL_UNUSED(context);
    // Perform reset.
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 0);
    _check_mcal_status();
    mcal_status = LPTIM_delay_milliseconds(SX126X_HAL_WAKEUP_DELAY_MS, LPTIM_DELAY_MODE_ACTIVE);
    _check_mcal_status();
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 1);
    _check_mcal_status();
errors:
    return status;
}

/*******************************************************************/
sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length, const uint8_t *data, const uint16_t data_length) {
    // Local variables.
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Ignore unused parameters.
    MCAL_UNUSED(context);
    // Wait for chip to be ready.
    status = _sx126x_hal_wait_busy();
    if (status != SX126X_HAL_STATUS_OK) {
        goto errors;
    }
    // SPI transfer.
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 0);
    _check_mcal_status();
    mcal_status = SPI_write_read((uint8_t *) command, NULL, command_length);
    _check_mcal_status();
    mcal_status = SPI_write_read((uint8_t *) data, NULL, data_length);
    _check_mcal_status();
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 1);
    _check_mcal_status();
errors:
    return status;
}

/*******************************************************************/
sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length, uint8_t *data, const uint16_t data_length) {
    // Local variables.
    sx126x_hal_status_t status = SX126X_HAL_STATUS_OK;
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Ignore unused parameters.
    MCAL_UNUSED(context);
    // Wait for chip to be ready.
    status = _sx126x_hal_wait_busy();
    if (status != SX126X_HAL_STATUS_OK) {
        goto errors;
    }
    // SPI transfer.
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 0);
    _check_mcal_status();
    mcal_status = SPI_write_read((uint8_t *) command, NULL, command_length);
    _check_mcal_status();
    mcal_status = SPI_write_read(NULL, data, data_length);
    _check_mcal_status();
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 1);
    _check_mcal_status();
errors:
    return status;
}
