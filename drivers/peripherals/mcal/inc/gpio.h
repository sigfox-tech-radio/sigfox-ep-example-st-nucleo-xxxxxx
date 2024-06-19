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

/*** GPIO macros ***/

#define GPIO_PINS_PER_PORT	16

/*** GPIO structures ***/

/*!******************************************************************
 * \struct GPIO_port_t
 * \brief GPIO ports list.
 *******************************************************************/
typedef enum {
	GPIO_PORT_A = 0,
	GPIO_PORT_B,
	GPIO_PORT_C,
	GPIO_PORT_LAST
} GPIO_port_t;

/*!******************************************************************
 * \struct GPIO_pin_t
 * \brief GPIO pin descriptor.
 *******************************************************************/
typedef struct {
	GPIO_port_t port;
	uint8_t pin;
	uint8_t alternate_function;
} GPIO_pin_t;

/*!******************************************************************
 * \enum GPIO_mode_t
 * \brief GPIO modes list.
 *******************************************************************/
typedef enum {
	GPIO_MODE_INPUT = 0,
	GPIO_MODE_OUTPUT,
	GPIO_MODE_ALTERNATE_FUNCTION,
	GPIO_MODE_ANALOG,
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
	GPIO_SPEED_LOW = 0,
	GPIO_SPEED_MEDIUM,
	GPIO_SPEED_HIGH,
	GPIO_SPEED_VERY_HIGH,
	GPIO_SPEED_LAST
} GPIO_speed_t;

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
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void GPIO_init(void);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_speed_t speed, GPIO_pull_resistor_t pull_resistor)
 * \brief Configure a GPIO.
 * \param[in]  	gpio: GPIO to configure.
 * \param[in]	mode: GPIO mode.
 * \param[in]	output_type: GPIO output type.
 * \param[in]	speed: GPIO speed.
 * \param[in]	pull_resistor: GPIO pull resistor configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_configure(const GPIO_pin_t* gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_speed_t speed, GPIO_pull_resistor_t pull_resistor);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_write(const GPIO_pin_t* gpio, uint8_t state)
 * \brief Set GPIO output state.
 * \param[in]  	gpio: GPIO to write.
 * \param[in]	state: Output state to write.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_write(const GPIO_pin_t* gpio, uint8_t state);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_read(const GPIO_pin_t* gpio, uint8_t* state)
 * \brief Read GPIO input or output state.
 * \param[in]  	gpio: GPIO to read.
 * \param[out] 	state: Pointer to the GPIO state.
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_read(const GPIO_pin_t* gpio, uint8_t* state);

/*!******************************************************************
 * \fn MCAL_status_t GPIO_toggle(const GPIO_pin_t* gpio)
 * \brief Toggle GPIO output state.
 * \param[in]  	gpio: GPIO to toggle.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t GPIO_toggle(const GPIO_pin_t* gpio);

#endif /* __GPIO_H__ */
