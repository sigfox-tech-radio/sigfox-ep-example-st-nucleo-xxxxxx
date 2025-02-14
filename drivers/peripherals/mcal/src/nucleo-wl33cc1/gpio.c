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
#include "stm32wl3x.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_gpio.h"

/*** GPIO local structures ***/

/*******************************************************************/
typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin;
} GPIO_def_t;

/*** GPIO local global variables ***/

static const GPIO_def_t GPIO_PIN_MAPPING[GPIO_PIN_LAST] = {
    // Digital pins.
    { GPIOB, LL_GPIO_PIN_7 },
    { GPIOB, LL_GPIO_PIN_6 },
    { GPIOA, LL_GPIO_PIN_9 },
    { GPIOA, LL_GPIO_PIN_0 },
    { GPIOA, LL_GPIO_PIN_8 },
    { GPIOA, LL_GPIO_PIN_13 },
    { GPIOA, LL_GPIO_PIN_12 },
    { GPIOA, LL_GPIO_PIN_4 },
    { GPIOA, LL_GPIO_PIN_5 },
    { GPIOA, LL_GPIO_PIN_14 },
    { GPIOB, LL_GPIO_PIN_10 },
    { GPIOB, LL_GPIO_PIN_9 },
    { GPIOB, LL_GPIO_PIN_8 },
    { GPIOB, LL_GPIO_PIN_11 },
    { GPIOA, LL_GPIO_PIN_7 },
    { GPIOA, LL_GPIO_PIN_6 },
    // Analog pins.
    { GPIOB, LL_GPIO_PIN_0 },
    { GPIOB, LL_GPIO_PIN_1 },
    { GPIOB, LL_GPIO_PIN_2 },
    { GPIOB, LL_GPIO_PIN_3 },
    { GPIOB, LL_GPIO_PIN_14 },
    { GPIOB, LL_GPIO_PIN_5 },
    // Push button.
    { GPIOB, LL_GPIO_PIN_15 },
    // LEDs.
    { GPIOB, LL_GPIO_PIN_5 },
    { GPIOB, LL_GPIO_PIN_4 },
    { GPIOA, LL_GPIO_PIN_14 },
    // Virtual COM port.
    { GPIOA, LL_GPIO_PIN_1 },
    { GPIOA, LL_GPIO_PIN_15 },
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

static const uint32_t GPIO_LL_SPEED[GPIO_OUTPUT_SPEED_LAST] = {
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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
}

/*******************************************************************/
MCAL_status_t GPIO_configure(GPIO_pin_t gpio, GPIO_mode_t mode, GPIO_output_type_t output_type, GPIO_output_speed_t speed, GPIO_pull_resistor_t pull_resistor, uint8_t af_number) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    LL_GPIO_InitTypeDef gpio_init;
    // Check parameters.
    if ((gpio >= GPIO_PIN_LAST) || (mode >= GPIO_MODE_LAST) || (output_type >= GPIO_OUTPUT_TYPE_LAST) || (speed >= GPIO_OUTPUT_SPEED_LAST) || (pull_resistor >= GPIO_PULL_LAST)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Check if pin is implemented.
    if (GPIO_PIN_MAPPING[gpio].port == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Configure pin.
    gpio_init.Pin = GPIO_PIN_MAPPING[gpio].pin;
    gpio_init.Mode = GPIO_LL_MODE[mode];
    gpio_init.Speed = GPIO_LL_SPEED[speed];
    gpio_init.OutputType = GPIO_LL_OUTPUT_TYPE[output_type];
    gpio_init.Pull = GPIO_LL_PULL[pull_resistor];
    gpio_init.Alternate = af_number;
    // Init pin.
    ll_status = LL_GPIO_Init(GPIO_PIN_MAPPING[gpio].port, &gpio_init);
    _check_ll_status();
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t GPIO_write(GPIO_pin_t gpio, uint8_t state) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameter.
    if (gpio >= GPIO_PIN_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Check if pin is implemented.
    if (GPIO_PIN_MAPPING[gpio].port == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    if (state == 0) {
        LL_GPIO_ResetOutputPin(GPIO_PIN_MAPPING[gpio].port, GPIO_PIN_MAPPING[gpio].pin);
    } else {
        LL_GPIO_SetOutputPin(GPIO_PIN_MAPPING[gpio].port, GPIO_PIN_MAPPING[gpio].pin);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t GPIO_read(GPIO_pin_t gpio, uint8_t *state) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameter.
    if ((gpio >= GPIO_PIN_LAST) && (state == NULL)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Check if pin is implemented.
    if (GPIO_PIN_MAPPING[gpio].port == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    if (LL_GPIO_GetPinMode(GPIO_PIN_MAPPING[gpio].port, GPIO_PIN_MAPPING[gpio].pin) == LL_GPIO_MODE_OUTPUT) {
        (*state) = LL_GPIO_IsOutputPinSet(GPIO_PIN_MAPPING[gpio].port, GPIO_PIN_MAPPING[gpio].pin);
    } else {
        (*state) = LL_GPIO_IsInputPinSet(GPIO_PIN_MAPPING[gpio].port, GPIO_PIN_MAPPING[gpio].pin);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t GPIO_toggle(GPIO_pin_t gpio) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameter.
    if (gpio >= GPIO_PIN_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Check if pin is implemented.
    if (GPIO_PIN_MAPPING[gpio].port == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    LL_GPIO_TogglePin(GPIO_PIN_MAPPING[gpio].port, GPIO_PIN_MAPPING[gpio].pin);
errors:
    return status;
}
