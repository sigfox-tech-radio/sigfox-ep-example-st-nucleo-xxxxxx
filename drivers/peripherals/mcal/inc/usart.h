/*!*****************************************************************
 * \file    usart.h
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

#ifndef __USART_H__
#define __USART_H__

#include "mcal.h"
#include "stdint.h"

/*** USART structures ***/

/*!******************************************************************
 * \fn USART_rx_irq_cb_t
 * \brief USART RX interrupt callback.
 *******************************************************************/
typedef void (*USART_rx_irq_cb_t)(uint8_t data);

/*** USART functions ***/

/*!******************************************************************
 * \fn USART_status_t USART_init(uint32_t baud_rate, USART_rx_irq_cb_t irq_callback)
 * \brief Init USART peripheral.
 * \param[in]   baud_rate: USART link baud rate.
 * \param[in]   irq_priority: RX interrupt priority.
 * \param[in]   irq_callback: Function to call on RX interrupt.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t USART_init(uint32_t baud_rate, uint8_t irq_priority, USART_rx_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn void USART_de_init(void)
 * \brief Release USART peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void USART_de_init(void);

/*!******************************************************************
 * \fn USART_status_t USART_write(uint8_t* data, uint32_t data_size_bytes)
 * \brief Send data over USART.
 * \param[in]   data: Byte array to send.
 * \param[in]   data_size_bytes: Number of bytes to send.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t USART_write(uint8_t *data, uint32_t data_size_bytes);

#endif /* __USART_H__ */
