/*!*****************************************************************
 * \file    sx126x_mapping.c
 * \brief   SX126X pins mapping.
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

#include "sx126x_mapping.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "gpio.h"

/*** SX126X MAPPING global variables ***/

const GPIO_pin_t SX126X_GPIO_NSS = 		(GPIO_pin_t) {GPIO_PORT_A, 8, 0};

const GPIO_pin_t SX126X_GPIO_NRESET = 	(GPIO_pin_t) {GPIO_PORT_A, 0, 0};

const GPIO_pin_t SX126X_GPIO_IRQ = 		(GPIO_pin_t) {GPIO_PORT_B, 4, 0};

const GPIO_pin_t SX126X_GPIO_BUSY =		(GPIO_pin_t) {GPIO_PORT_B, 3, 0};

const GPIO_pin_t SX126X_GPIO_ANT_SW =	(GPIO_pin_t) {GPIO_PORT_A, 9, 0};

const GPIO_pin_t SX126X_GPIO_LED_TX = 	(GPIO_pin_t) {GPIO_PORT_C, 1, 0};

const GPIO_pin_t SX126X_GPIO_LED_RX = 	(GPIO_pin_t) {GPIO_PORT_C, 0, 0};
