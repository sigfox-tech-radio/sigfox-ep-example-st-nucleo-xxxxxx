/*!*****************************************************************
 * \file   S2LP_CORE_SPI.c
 * \brief  S2LP library low level functions.
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

#include "S2LP_CORE_SPI.h"

#include "gpio.h"
#include "mcal.h"
#include "spi.h"
#include "S2LP_Types.h"
#include "s2lp_shield.h"
#include "stdint.h"

/*** S2LP HW local macros ***/

#define S2LP_CORE_SPI_BUFFER_SIZE_BYTES     128

#define S2LP_CORE_SPI_HEADER_BYTE_WRITE     0x00
#define S2LP_CORE_SPI_HEADER_BYTE_READ      0x01
#define S2LP_CORE_SPI_HEADER_BYTE_COMMAND   0x80

#define S2LP_CORE_SPI_FIFO_ADDRESS          0xFF

/*** S2LP HW local global variables ***/

static uint8_t tx_data[S2LP_CORE_SPI_BUFFER_SIZE_BYTES];
static uint8_t rx_data[S2LP_CORE_SPI_BUFFER_SIZE_BYTES];

/*******************************************************************/
uint8_t S2LPSpiWriteRegisters(uint8_t register_address, uint8_t data_size, uint8_t *data) {
    // Local variables.
    uint8_t idx = 0;
    // Build TX data.
    tx_data[0] = S2LP_CORE_SPI_HEADER_BYTE_WRITE;
    tx_data[1] = register_address;
    for (idx = 0; idx < data_size; idx++) {
        tx_data[idx + 2] = data[idx];
    }
    // SPI transfer.
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 0);
    SPI_write_read((uint8_t *) tx_data, (uint8_t *) rx_data, (data_size + 2));
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 1);
    // Return status byte
    return (rx_data[1]);
}

/*******************************************************************/
uint8_t S2LPSpiReadRegisters(uint8_t register_address, uint8_t data_size, uint8_t *data) {
    // Local variables.
    uint8_t idx = 0;
    // Build TX data.
    tx_data[0] = S2LP_CORE_SPI_HEADER_BYTE_READ;
    tx_data[1] = register_address;
    for (idx = 0; idx < data_size; idx++) {
        tx_data[idx + 2] = 0x00;
    }
    // SPI transfer.
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 0);
    SPI_write_read((uint8_t *) tx_data, (uint8_t *) rx_data, (data_size + 2));
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 1);
    // Fill RX buffer.
    for (idx = 0; idx < data_size; idx++) {
        data[idx] = rx_data[idx + 2];
    }
    // Return status byte
    return (rx_data[1]);
}

/*******************************************************************/
uint8_t S2LPSpiCommandStrobes(uint8_t commmand) {
    // Build TX data.
    tx_data[0] = S2LP_CORE_SPI_HEADER_BYTE_COMMAND;
    tx_data[1] = commmand;
    // SPI transfer.
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 0);
    SPI_write_read((uint8_t *) tx_data, (uint8_t *) rx_data, 2);
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 1);
    // Return status byte
    return (rx_data[1]);
}

/*******************************************************************/
uint8_t S2LPSpiWriteFifo(uint8_t data_size, uint8_t *data) {
    // Local variables.
    uint8_t idx = 0;
    // Build TX data.
    tx_data[0] = S2LP_CORE_SPI_HEADER_BYTE_WRITE;
    tx_data[1] = S2LP_CORE_SPI_FIFO_ADDRESS;
    for (idx = 0; idx < data_size; idx++) {
        tx_data[idx + 2] = data[idx];
    }
    // SPI transfer.
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 0);
    SPI_write_read((uint8_t *) tx_data, (uint8_t *) rx_data, (data_size + 2));
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 1);
    // Return status byte
    return (rx_data[1]);
}

/*******************************************************************/
uint8_t S2LPSpiReadFifo(uint8_t data_size, uint8_t *data) {
    // Local variables.
    uint8_t byte_idx = 0;
    // Build TX data.
    tx_data[0] = S2LP_CORE_SPI_HEADER_BYTE_READ;
    tx_data[1] = S2LP_CORE_SPI_FIFO_ADDRESS;
    for (byte_idx = 0; byte_idx < data_size; byte_idx++) {
        tx_data[byte_idx + 2] = 0x00;
    }
    // SPI transfer.
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 0);
    SPI_write_read((uint8_t *) tx_data, (uint8_t *) rx_data, (data_size + 2));
    GPIO_write(S2LP_SHIELD_GPIO.spi_nss, 1);
    // Fill RX buffer.
    for (byte_idx = 0; byte_idx < data_size; byte_idx++) {
        data[byte_idx] = rx_data[byte_idx + 2];
    }
    // Return status byte
    return (rx_data[1]);
}
