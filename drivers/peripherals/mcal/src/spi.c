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
#include "gpio.h"
#include "stdint.h"
#include "stddef.h"
#ifdef STM32L0XX
#include "stm32l0xx.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_spi.h"
#endif

/*** SPI local macros ***/

#define SPI_INSTANCE				SPI1
#define SPI_ACCESS_TIMEOUT_COUNT	1000000

/*** SPI local global variables ***/

static const GPIO_pin_t SPI_GPIO_SCK = 	(GPIO_pin_t) {GPIO_PORT_A, 5, 0};
static const GPIO_pin_t SPI_GPIO_MISO = (GPIO_pin_t) {GPIO_PORT_A, 6, 0};
static const GPIO_pin_t SPI_GPIO_MOSI = (GPIO_pin_t) {GPIO_PORT_A, 7, 0};

/*** SPI local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*** SPI functions ***/

/*******************************************************************/
MCAL_status_t SPI_init(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	ErrorStatus ll_status = SUCCESS;
	LL_SPI_InitTypeDef spi_init;
	// Enable peripheral clock.
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	// Configure SPI.
	spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
	spi_init.Mode = LL_SPI_MODE_MASTER;
	spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
	spi_init.ClockPolarity = LL_SPI_POLARITY_LOW;
	spi_init.ClockPhase = LL_SPI_PHASE_1EDGE;
	spi_init.NSS = LL_SPI_NSS_SOFT;
	spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
	spi_init.BitOrder = LL_SPI_MSB_FIRST;
	spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
	spi_init.CRCPoly = 7U;
	// Init SPI.
	ll_status = LL_SPI_Init(SPI_INSTANCE, &spi_init);
	_check_ll_status();
	// Init GPIOs.
	status = GPIO_configure(&SPI_GPIO_SCK,  GPIO_MODE_ALTERNATE_FUNCTION, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
	status = GPIO_configure(&SPI_GPIO_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
	status = GPIO_configure(&SPI_GPIO_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
	// Enable SPI.
	LL_SPI_Enable(SPI_INSTANCE);
errors:
	return status;
}

/*******************************************************************/
MCAL_status_t SPI_de_init(void) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	// Put GPIOs in high impedance.
	status = GPIO_configure(&SPI_GPIO_SCK,  GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
	status = GPIO_configure(&SPI_GPIO_MISO, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
	status = GPIO_configure(&SPI_GPIO_MOSI, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	if (status != MCAL_SUCCESS) goto errors;
errors:
	// Disable SPI and clock.
	LL_SPI_Disable(SPI_INSTANCE);
	LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SPI1);
	return status;
}

/*******************************************************************/
MCAL_status_t SPI_write_read(uint8_t* tx_data, uint8_t* rx_data, uint16_t transfer_size) {
	// Local variables.
	MCAL_status_t status = MCAL_SUCCESS;
	uint8_t transfer_idx = 0;
	uint8_t data_byte = 0;
	uint32_t loop_count = 0;
	// Transfer loop.
	for (transfer_idx=0 ; transfer_idx<transfer_size ; transfer_idx++) {
		// Dummy read to DR to clear RXNE flag.
		data_byte = LL_SPI_ReceiveData8(SPI_INSTANCE);
		// Wait for TXE flag.
		loop_count = 0;
		while (LL_SPI_IsActiveFlag_TXE(SPI_INSTANCE) == 0) {
			// Wait for TXE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = MCAL_SUCCESS;
				goto errors;
			}
		}
		// Send TX byte.
		data_byte = ((tx_data != NULL) ? tx_data[transfer_idx] : 0x00);
		LL_SPI_TransmitData8(SPI_INSTANCE, data_byte);
		// Wait for incoming data.
		loop_count = 0;
		while (LL_SPI_IsActiveFlag_RXNE(SPI_INSTANCE) == 0) {
			// Wait for RXNE='1' or timeout.
			loop_count++;
			if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) {
				status = MCAL_SUCCESS;
				goto errors;
			}
		}
		data_byte = LL_SPI_ReceiveData8(SPI_INSTANCE);
		if (rx_data != NULL) rx_data[transfer_idx] = data_byte;
	}
errors:
	return status;
}
