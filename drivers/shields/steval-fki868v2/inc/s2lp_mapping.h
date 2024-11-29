/*!*****************************************************************
 * \file    s2lp_mapping.h
 * \brief   S2LP pins mapping.
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

#ifndef __S2LP_MAPPING_H__
#define __S2LP_MAPPING_H__

#include "gpio.h"

/*** S2LP MAPPING macros ***/

#define S2LP_GPIO_IRQ_EXTI_PORT     EXTI_PORT_C
#define S2LP_GPIO_IRQ_EXTI_LINE     EXTI_LINE_GPIO_0

/*** S2LP MAPPING global variables ***/

extern const GPIO_pin_t S2LP_GPIO_SPI_SCK;
extern const GPIO_pin_t S2LP_GPIO_SPI_MISO;
extern const GPIO_pin_t S2LP_GPIO_SPI_MOSI;
extern const GPIO_pin_t S2LP_GPIO_SPI_NSS;

extern const GPIO_pin_t S2LP_GPIO_SDN;

extern const GPIO_pin_t S2LP_GPIO_IRQ;

#endif /* __S2LP_MAPPING_H__ */
