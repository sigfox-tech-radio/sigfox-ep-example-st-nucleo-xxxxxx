/*!*****************************************************************
 * \file    gpio.h
 * \brief   Common GPIO driver based on LL driver.
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

#ifndef __GPIO_H__
#define __GPIO_H__

#include "mcal.h"
#include "stdint.h"

/*** GPIO structures ***/

/*!******************************************************************
 * \enum GPIO_pin_t
 * \brief Unified Nucleo pins name.
 *******************************************************************/
typedef enum {
    // Digital pins.
    GPIO_PIN_D0 = 0,
    GPIO_PIN_D1,
    GPIO_PIN_D2,
    GPIO_PIN_D3,
    GPIO_PIN_D4,
    GPIO_PIN_D5,
    GPIO_PIN_D6,
    GPIO_PIN_D7,
    GPIO_PIN_D8,
    GPIO_PIN_D9,
    GPIO_PIN_D10,
    GPIO_PIN_D11,
    GPIO_PIN_D12,
    GPIO_PIN_D13,
    GPIO_PIN_D14,
    GPIO_PIN_D15,
    // Analog pins.
    GPIO_PIN_A0,
    GPIO_PIN_A1,
    GPIO_PIN_A2,
    GPIO_PIN_A3,
    GPIO_PIN_A4,
    GPIO_PIN_A5,
    // Push button.
    GPIO_PIN_BP_USER,
    // LEDs.
    GPIO_PIN_LED_RED,
    GPIO_PIN_LED_GREEN,
    GPIO_PIN_LED_BLUE,
    // Virtual COM port.
    GPIO_PIN_VCP_TX,
    GPIO_PIN_VCP_RX,
    // Last index.
    GPIO_PIN_LAST
} GPIO_pin_t;

/*!******************************************************************
 * \enum GPIO_mode_t
 * \brief GPIO modes list.
 *******************************************************************/
typedef enum {
    GPIO_MODE_DIGITAL_INPUT = 0,
    GPIO_MODE_DIGITAL_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG_INPUT,
    GPIO_MODE_LAST
} GPIO_mode_t;

/*!******************************************************************
 * \enum GPIO_output_type_t
 * \brief GPIO output types.
 *******************************************************************/
typedef enum {
    GPIO_OUTPUT_TYPE_PUSH_PULL = 0,
    GPIO_OUTPUT_TYPE_OPEN_DRAIN,
    GPIO_OUTPUT_TYPE_LAST
} GPIO_output_type_t;

/*!******************************************************************
 * \enum GPIO_speed_t
 * \brief GPIO speeds.
 *******************************************************************/
typedef enum {
    GPIO_OUTPUT_SPEED_LOW = 0,
    GPIO_OUTPUT_SPEED_MEDIUM,
    GPIO_OUTPUT_SPEED_HIGH,
    GPIO_OUTPUT_SPEED_VERY_HIGH,
    GPIO_OUTPUT_SPEED_LAST
} GPIO_output_speed_t;

/*!******************************************************************
 * \enum GPIO_pull_resistor_t
 * \brief GPIO pull resistor configurations.
 *******************************************************************/
typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN,
    GPIO_PULL_LAST
} GPIO_pull_resistor_t;

/*** GPIO functions ***/

/*!******************************************************************
 * \fn void GPIO_init(void)
 * \brief Init GPIO driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void GPIO_init(void);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_configure(GPIO_pin_t gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t speed, GPIO_pull_resistor_t pull_resistor, uint8_t af_number)
 * \brief Configure a GPIO.
 * \param[in]   gpio: GPIO to configure.
 * \param[in]   mode: GPIO mode.
 * \param[in]   output_type: GPIO output type.
 * \param[in]   speed: GPIO speed.
 * \param[in]   pull_resistor: GPIO pull resistor configuration.
 * \param[in]   af_number: Alternate function number (only when relevant mode is selected).
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_configure(GPIO_pin_t gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t speed, GPIO_pull_resistor_t pull_resistor, uint8_t af_number);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_write(GPIO_pin_t gpio, uint8_t state)
 * \brief Set GPIO output state.
 * \param[in]   gpio: GPIO to write.
 * \param[in]   state: Output state to write.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_write(GPIO_pin_t gpio, uint8_t state);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_read(GPIO_pin_t gpio, uint8_t* state)
 * \brief Read GPIO input or output state.
 * \param[in]   gpio: GPIO to read.
 * \param[out]  state: Pointer to the GPIO state.
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_read(GPIO_pin_t gpio, uint8_t *state);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_toggle(GPIO_pin_t gpio)
 * \brief Toggle GPIO output state.
 * \param[in]   gpio: GPIO to toggle.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_toggle(GPIO_pin_t gpio);

#endif /* __GPIO_H__ */
