/*!*****************************************************************
 * \file    usart.c
 * \brief   Common USART driver based on LL driver.
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

#include "usart.h"

#include "exti.h"
#include "gpio.h"
#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#ifdef STM32L0XX
#include "stm32l0xx.h"
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_usart.h"
#endif

/*** USART local macros ***/

#define USART_INSTANCE          USART2
#define USART_IRQN              USART2_IRQn

#define USART_TIMEOUT_COUNT     100000

/*** SPI local global variables ***/

static const GPIO_pin_t USART_GPIO_TX = { GPIO_PORT_A, 2, 4 };
static const GPIO_pin_t USART_GPIO_RX = { GPIO_PORT_A, 3, 4 };

/*** SPI local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*** USART local global variables ***/

static USART_rx_irq_cb_t usart_rx_irq_callback = NULL;

/*** USART local functions ***/

/*******************************************************************/
void __attribute__((optimize("-O0"))) USART2_IRQHandler(void) {
    // Local variables.
    uint8_t rx_byte = 0;
    // RXNE interrupt.
    if (LL_USART_IsActiveFlag_RXNE(USART_INSTANCE) != 0) {
        // Read incoming byte.
        rx_byte = LL_USART_ReceiveData8(USART_INSTANCE);
        // Transmit byte to upper layer.
        if ((LL_USART_IsEnabledIT_RXNE(USART_INSTANCE) != 0) && (usart_rx_irq_callback != NULL)) {
            usart_rx_irq_callback(rx_byte);
        }
    }
    // Overrun error interrupt.
    if (LL_USART_IsActiveFlag_ORE(USART_INSTANCE) != 0) {
        // Clear ORE flag.
        LL_USART_ClearFlag_ORE(USART_INSTANCE);
    }
    EXTI_clear_flag(EXTI_LINE_USART2);
}

/*** USART functions ***/

/*******************************************************************/
MCAL_status_t USART_init(uint32_t baud_rate, uint8_t irq_priority, USART_rx_irq_cb_t irq_callback) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    LL_USART_InitTypeDef usart_init;
    // Select HSI as peripheral clock.
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_HSI);
    // Enable peripheral clock.
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB1_GRP1_EnableClockSleep(LL_APB1_GRP1_PERIPH_USART2);
    LL_USART_EnableClockInStopMode(USART_INSTANCE);
    // Configure peripheral.
    usart_init.BaudRate = baud_rate;
    usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
    usart_init.StopBits = LL_USART_STOPBITS_1;
    usart_init.Parity = LL_USART_PARITY_NONE;
    usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
    usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
    ll_status = LL_USART_Init(USART_INSTANCE, &usart_init);
    _check_ll_status();
    LL_USART_DisableOverrunDetect(USART_INSTANCE);
    // Configure interrupt.
    LL_USART_EnableIT_RXNE(USART_INSTANCE);
    // Enable peripheral.
    LL_USART_EnableInStopMode(USART_INSTANCE);
    LL_USART_Enable(USART_INSTANCE);
    // Configure GPIOs.
    GPIO_configure(&USART_GPIO_TX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&USART_GPIO_RX, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Register callback.
    usart_rx_irq_callback = irq_callback;
    // Enable interrupt.
    // Enable interrupt.
    NVIC_SetPriority(USART_IRQN, irq_priority);
    NVIC_EnableIRQ(USART_IRQN);
errors:
    return status;
}

/*******************************************************************/
void USART_de_init(void) {
    // Disable interrupt.
    NVIC_DisableIRQ(USART_IRQN);
    // Disable USART alternate function.
    GPIO_configure(&USART_GPIO_TX, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(&USART_GPIO_RX, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    // Disable peripheral.
    LL_USART_Disable(USART_INSTANCE);
    // Disable peripheral clock.
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_USART2);
}

/*******************************************************************/
MCAL_status_t USART_write(uint8_t *data, uint32_t data_size_bytes) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint8_t idx = 0;
    uint32_t loop_count = 0;
    // Check parameters.
    if (data == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Byte loop.
    for (idx = 0; idx < data_size_bytes; idx++) {
        // Fill transmit register.
        LL_USART_TransmitData8(USART_INSTANCE, data[idx]);
        // Wait for transmission to complete.
        while (LL_USART_IsActiveFlag_TXE(USART_INSTANCE) == 0) {
            // Wait for TXE='1' or timeout.
            loop_count++;
            if (loop_count > USART_TIMEOUT_COUNT) {
                status = MCAL_ERROR;
                goto errors;
            }
        }
    }
errors:
    return status;
}
