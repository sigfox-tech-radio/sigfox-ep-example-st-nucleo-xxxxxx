/*!*****************************************************************
 * \file    gpio.c
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

#include "gpio.h"

#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#ifdef STM32L0XX
#include "stm32l0xx.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_gpio.h"
#endif

/*** GPIO local global variables ***/

static GPIO_TypeDef *const GPIO_LL_PORT[GPIO_PORT_LAST] = {
    GPIOA,
    GPIOB,
    GPIOC,
};
static const uint32_t GPIO_LL_MODE[GPIO_MODE_LAST] = {
    LL_GPIO_MODE_INPUT,
    LL_GPIO_MODE_OUTPUT,
    LL_GPIO_MODE_ALTERNATE,
    LL_GPIO_MODE_ANALOG,
};
static const uint32_t GPIO_LL_OUTPUT_TYPE[GPIO_OUTPUT_TYPE_LAST] = {
    LL_GPIO_OUTPUT_PUSHPULL,
    LL_GPIO_OUTPUT_OPENDRAIN,
};
static const uint32_t GPIO_LL_SPEED[GPIO_SPEED_LAST] = {
    LL_GPIO_SPEED_FREQ_LOW,
    LL_GPIO_SPEED_FREQ_MEDIUM,
    LL_GPIO_SPEED_FREQ_HIGH,
    LL_GPIO_SPEED_FREQ_VERY_HIGH,
};
static const uint32_t GPIO_LL_PULL[GPIO_PULL_LAST] = {
    LL_GPIO_PULL_NO,
    LL_GPIO_PULL_UP,
    LL_GPIO_PULL_DOWN,
};

/*** GPIO local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*** GPIO functions ***/

/*******************************************************************/
void GPIO_init(void) {
    // Enable all GPIOs clocks.
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
}

/*******************************************************************/
MCAL_status_t GPIO_configure(const GPIO_pin_t *gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_speed_t speed, GPIO_pull_resistor_t pull_resistor) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    LL_GPIO_InitTypeDef gpio_init;
    // Check parameters.
    if ((gpio == NULL) || (mode >= GPIO_MODE_LAST) || (output_type >= GPIO_OUTPUT_TYPE_LAST) || (speed >= GPIO_SPEED_LAST) || (pull_resistor >= GPIO_PULL_LAST)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Configure pin.
    gpio_init.Pin = (0b1 << (gpio->pin));
    gpio_init.Mode = GPIO_LL_MODE[mode];
    gpio_init.Speed = GPIO_LL_SPEED[speed];
    gpio_init.OutputType = GPIO_LL_OUTPUT_TYPE[output_type];
    gpio_init.Pull = GPIO_LL_PULL[pull_resistor];
    gpio_init.Alternate = (gpio->alternate_function);
    // Init pin.
    ll_status = LL_GPIO_Init(GPIO_LL_PORT[gpio->port], &gpio_init);
    _check_ll_status();
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t GPIO_write(const GPIO_pin_t *gpio, uint8_t state) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t ll_pin = 0;
    // Check parameter.
    if (gpio == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    ll_pin = (0b1 << (gpio->pin));
    if (state == 0) {
        LL_GPIO_ResetOutputPin(GPIO_LL_PORT[gpio->port], ll_pin);
    } else {
        LL_GPIO_SetOutputPin(GPIO_LL_PORT[gpio->port], ll_pin);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t GPIO_read(const GPIO_pin_t *gpio, uint8_t *state) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t ll_pin = 0;
    // Check parameter.
    if ((gpio == NULL) && (state == NULL)) {
        status = MCAL_ERROR;
        goto errors;
    }
    ll_pin = (0b1 << (gpio->pin));
    if (LL_GPIO_GetPinMode(GPIO_LL_PORT[gpio->port], ll_pin) == LL_GPIO_MODE_OUTPUT) {
        (*state) = LL_GPIO_IsOutputPinSet(GPIO_LL_PORT[gpio->port], ll_pin);
    } else {
        (*state) = LL_GPIO_IsInputPinSet(GPIO_LL_PORT[gpio->port], ll_pin);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t GPIO_toggle(const GPIO_pin_t *gpio) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameter.
    if (gpio == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    LL_GPIO_TogglePin(GPIO_LL_PORT[gpio->port], (0b1 << (gpio->pin)));
errors:
    return status;
}
