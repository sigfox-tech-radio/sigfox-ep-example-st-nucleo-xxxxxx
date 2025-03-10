/*!*****************************************************************
 * \file    spi.c
 * \brief   Common SPI driver based on LL driver.
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

#include "spi.h"

#include "mcal.h"
#include "stdint.h"

/*** SPI functions ***/

/*******************************************************************/
MCAL_status_t SPI_init(SPI_gpio_t *spi_gpio) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(spi_gpio);
    return status;
}

/*******************************************************************/
MCAL_status_t SPI_de_init(SPI_gpio_t *spi_gpio) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(spi_gpio);
    return status;
}

/*******************************************************************/
MCAL_status_t SPI_write_read(uint8_t *tx_data, uint8_t *rx_data, uint16_t transfer_size) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(tx_data);
    MCAL_UNUSED(rx_data);
    MCAL_UNUSED(transfer_size);
    return status;
}
