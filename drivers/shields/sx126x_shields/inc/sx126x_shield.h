/*!*****************************************************************
 * \file    sx126x_shield.h
 * \brief   SX126X shields common definitions.
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

#ifndef __SX126X_SHIELD_H__
#define __SX126X_SHIELD_H__

#include "gpio.h"

/*** SX126X SHIELD macros ***/

#define SX126X_HAL_RESET_DELAY_MS       2
#define SX126X_HAL_WAKEUP_DELAY_MS      2

#define SX126X_SHIELDS_SX1261_MIN_PWR   -17
#define SX126X_SHIELDS_SX1261_MAX_PWR   15

/*** SX126X SHIELD structures ***/

/*!******************************************************************
 * \struct SX126X_shield_gpio_t
 * \brief SX126X shield GPIOs mapping.
 *******************************************************************/
typedef struct {
    GPIO_pin_t spi_sck;
    GPIO_pin_t spi_miso;
    GPIO_pin_t spi_mosi;
    GPIO_pin_t spi_nss;
    GPIO_pin_t busy;
    GPIO_pin_t irq;
    GPIO_pin_t reset;
    GPIO_pin_t antenna_sw;
    GPIO_pin_t led_tx;
    GPIO_pin_t led_rx;
} SX126X_shield_gpio_t;

/*** SX126X SHIELD global variables ***/

extern const SX126X_shield_gpio_t SX126X_SHIELD_GPIO;

#endif /* __SX126X_SHIELD_H__ */
