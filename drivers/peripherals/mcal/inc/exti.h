/*!*****************************************************************
 * \file    exti.h
 * \brief   Common EXTI driver based on LL driver.
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

#ifndef __EXTI_H__
#define __EXTI_H__

#include "mcal.h"
#include "stdint.h"

/*** EXTI structures ***/

/*!******************************************************************
 * \enum EXTI_port_t
 * \brief EXTI ports list.
 *******************************************************************/
typedef enum {
    EXTI_PORT_A = 0,
    EXTI_PORT_B,
    EXTI_PORT_C,
    EXTI_PORT_LAST,
    EXTI_PORT_NONE = 0xFF
} EXTI_port_t;

/*!******************************************************************
 * \enum EXTI_line_t
 * \brief EXTI lines list.
 *******************************************************************/
typedef enum {
    EXTI_LINE_GPIO_0 = 0,
    EXTI_LINE_GPIO_1 = 1,
    EXTI_LINE_GPIO_2 = 2,
    EXTI_LINE_GPIO_3 = 3,
    EXTI_LINE_GPIO_4 = 4,
    EXTI_LINE_GPIO_5 = 5,
    EXTI_LINE_GPIO_6 = 6,
    EXTI_LINE_GPIO_7 = 7,
    EXTI_LINE_GPIO_8 = 8,
    EXTI_LINE_GPIO_9 = 9,
    EXTI_LINE_GPIO_10 = 10,
    EXTI_LINE_GPIO_11 = 11,
    EXTI_LINE_GPIO_12 = 12,
    EXTI_LINE_GPIO_13 = 13,
    EXTI_LINE_GPIO_14 = 14,
    EXTI_LINE_GPIO_15 = 15,
    EXTI_LINE_PVD = 16,
    EXTI_LINE_RTC_ALARM = 17,
    EXTI_LINE_RTC_TAMPER_TIMESTAMP = 19,
    EXTI_LINE_RTC_WAKEUP_TIMER = 20,
    EXTI_LINE_COMP1 = 21,
    EXTI_LINE_COMP2 = 22,
    EXTI_LINE_I2C1 = 23,
    EXTI_LINE_USART1 = 25,
    EXTI_LINE_USART2 = 26,
    EXTI_LINE_LPUART1 = 28,
    EXTI_LINE_LPTIM1 = 29,
    EXTI_LINE_LAST
} EXTI_line_t;

/*!******************************************************************
 * \enum EXTI_trigger_t
 * \brief EXTI triggers list.
 *******************************************************************/
typedef enum {
    EXTI_TRIGGER_NONE = 0,
    EXTI_TRIGGER_RISING,
    EXTI_TRIGGER_FALLING,
    EXTI_TRIGGER_RISING_FALLING,
    EXTI_TRIGGER_LAST
} EXTI_trigger_t;

/*!******************************************************************
 * \fn EXTI_gpio_irq_cb_t
 * \brief EXTI GPIO callback.
 *******************************************************************/
typedef void (*EXTI_gpio_irq_cb_t)(void);

/*** EXTI functions ***/

/*!******************************************************************
 * \fn void EXTI_init(void)
 * \brief Init common EXTI peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
void EXTI_init(void);

/*!******************************************************************
 * \fn void EXTI_de_init(void)
 * \brief Release common EXTI peripheral.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
void EXTI_de_init(void);

/*!******************************************************************
 * \fn MCAL_status_t EXTI_configure(EXTI_port_t port, EXTI_line_t line, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback)
 * \brief Init EXTI line.
 * \param[in]   port: Source port to connect.
 * \param[in]   line: Interrupt line to configure.
 * \param[in]   trigger: Signal trigger direction.
 * \param[in]   irq_callback: Function to call on interrupt (in case of GPIO).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t EXTI_configure(EXTI_port_t port, EXTI_line_t line, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback);

/*!******************************************************************
 * \fn MCAL_status_t EXTI_de_configure(EXTI_line_t line)
 * \brief Disable EXTI line.
 * \param[in]   line: Interrupt line to disable.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t EXTI_de_configure(EXTI_line_t line);

/*!******************************************************************
 * \fn MCAL_status_t EXTI_enable_irq(EXTI_line_t line, uint8_t priority)
 * \brief Enable EXTI line interrupt.
 * \param[in]   line: Interrupt line to configure.
 * \param[in]   irq_priority: Interrupt priority.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t EXTI_enable_irq(EXTI_line_t line, uint8_t priority);

/*!******************************************************************
 * \fn MCAL_status_t EXTI_disable_irq(EXTI_line_t line)
 * \brief Disable EXTI line interrupt.
 * \param[in]   line: Interrupt line to disable.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t EXTI_disable_irq(EXTI_line_t line);

/*!******************************************************************
 * \fn MCAL_status_t EXTI_clear_flag(EXTI_line_t line)
 * \brief Clear EXTI line flag.
 * \param[in]   line: Interrupt line to clear.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t EXTI_clear_flag(EXTI_line_t line);

#endif /* __EXTI_H__ */
