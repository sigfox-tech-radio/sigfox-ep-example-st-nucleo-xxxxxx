/*!*****************************************************************
 * \file    exti.c
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

#include "exti.h"

#include "gpio.h"
#include "mcal.h"
#include "stdint.h"
#include "stddef.h"
#include "stm32wl3x.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_pwr.h"
#include "stm32wl3x_ll_system.h"

/*** EXTI local macros ***/

#define EXTI_GPIO_PINS_PER_PORT     16

/*** EXTI local structures ***/

/*******************************************************************/
typedef enum {
    EXTI_PORT_A = 0,
    EXTI_PORT_B,
    EXTI_PORT_LAST,
} EXTI_port_t;

/*******************************************************************/
typedef struct {
    EXTI_port_t port;
    uint32_t line;
    uint8_t pin;
} EXTI_def_t;

/*******************************************************************/
typedef struct {
    EXTI_gpio_irq_cb_t gpio_irq_callbacks[EXTI_PORT_LAST][EXTI_GPIO_PINS_PER_PORT];
    uint8_t gpioa_irq_enable_count;
    uint8_t gpiob_irq_enable_count;
} EXTI_context_t;

/*** EXTI local global variables ***/

static const EXTI_def_t EXTI_PIN_MAPPING[GPIO_PIN_LAST] = {
    // Digital pins.
    { EXTI_PORT_B, LL_EXTI_LINE_PB7, 7 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB6, 6 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA9, 9 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA0, 0 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA8, 8 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA13, 13 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA12, 12 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA4, 4 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA5, 5 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA14, 14 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB10, 10 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB9, 9 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB8, 8 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB11, 11 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA7, 7 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA6, 6 },
    // Analog pins.
    { EXTI_PORT_B, LL_EXTI_LINE_PB0, 0 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB1, 1 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB2, 2 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB3, 3 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB14, 14 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB5, 5 },
    // Push button.
    { EXTI_PORT_B, LL_EXTI_LINE_PB15, 15 },
    // LEDs.
    { EXTI_PORT_B, LL_EXTI_LINE_PB5, 5 },
    { EXTI_PORT_B, LL_EXTI_LINE_PB4, 4 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA14, 14 },
    // Virtual COM port.
    { EXTI_PORT_A, LL_EXTI_LINE_PA1, 1 },
    { EXTI_PORT_A, LL_EXTI_LINE_PA15, 15 },
};

static const uint8_t EXTI_LL_TRIGGER[EXTI_TRIGGER_LAST] = {
    LL_EXTI_TRIGGER_NONE,
    LL_EXTI_TRIGGER_RISING_EDGE,
    LL_EXTI_TRIGGER_FALLING_EDGE,
    LL_EXTI_TRIGGER_BOTH_EDGE,
};

static const uint32_t EXTI_PWR_PORT[EXTI_PORT_LAST] = {
    LL_PWR_WAKEUP_PORTA,
    LL_PWR_WAKEUP_PORTB
};

static EXTI_context_t exti_ctx;

/*** EXTI local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
static void _EXTI_irq_handler(EXTI_port_t port) {
    // Local variables.
    uint8_t idx = 0;
    uint32_t line = 0;
    // Loop on all pins of port A.
    for (idx = 0; idx < EXTI_GPIO_PINS_PER_PORT; idx++) {
        // Convert to line.
        line = (1 << (idx + (port << 4)));
        // Check flag.
        if (LL_EXTI_IsActiveFlag(line) != 0) {
            // Clear flag.
            LL_EXTI_ClearFlag(line);
            // Call callback.
            if (exti_ctx.gpio_irq_callbacks[port][idx] != NULL) {
                exti_ctx.gpio_irq_callbacks[port][idx]();
            }
        }
    }
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) GPIOA_IRQHandler(void) {
    _EXTI_irq_handler(EXTI_PORT_A);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) GPIOB_IRQHandler(void) {
    _EXTI_irq_handler(EXTI_PORT_B);
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
    // Local variables.
    uint8_t idx = 0;
    uint8_t jdx = 0;
    // Enable peripheral clock.
    LL_APB0_GRP1_EnableClock(LL_APB0_GRP1_PERIPH_SYSCFG);
    LL_SYSCFG_PWRC_EnableIT(LL_SYSCFG_PWRC_WKUP);
    // Init context.
    for (idx = 0; idx < EXTI_PORT_LAST; idx++) {
        for (jdx = 0; jdx < EXTI_GPIO_PINS_PER_PORT; jdx++) {
            exti_ctx.gpio_irq_callbacks[idx][jdx] = NULL;
        }
    }
    exti_ctx.gpioa_irq_enable_count = 0;
    exti_ctx.gpiob_irq_enable_count = 0;
}

/*******************************************************************/
void EXTI_de_init(void) {
    // Disable peripheral clock.
    LL_APB0_GRP1_DisableClock(LL_APB0_GRP1_PERIPH_SYSCFG);
}

/*******************************************************************/
MCAL_status_t EXTI_configure_line(EXTI_line_t line, uint8_t state, EXTI_trigger_t trigger) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(line);
    MCAL_UNUSED(state);
    MCAL_UNUSED(trigger);
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_configure_gpio(GPIO_pin_t gpio, uint8_t state, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    EXTI_port_t port = EXTI_PORT_LAST;
    uint32_t line = 0;
    uint8_t pin = 0;
    LL_SYSCFG_IO_InitTypeDef io_init;
    // Check parameters.
    if ((gpio >= GPIO_PIN_LAST) || (trigger >= EXTI_TRIGGER_LAST)) {
        status = MCAL_ERROR;
        goto errors;
    }
    port = EXTI_PIN_MAPPING[gpio].port;
    line = EXTI_PIN_MAPPING[gpio].line;
    pin = EXTI_PIN_MAPPING[gpio].pin;
    // Init IO.
    io_init.Line = line;
    io_init.LineCommand = ((state != 0) ? ENABLE : DISABLE);
    io_init.Type = LL_EXTI_TYPE_EDGE;
    io_init.Trigger = ((state != 0) ? EXTI_LL_TRIGGER[trigger] : LL_EXTI_TRIGGER_NONE);
    ll_status = LL_SYSCFG_IO_Init(&io_init);
    _check_ll_status();
    // Register callback.
    exti_ctx.gpio_irq_callbacks[port][pin] = irq_callback;
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_set_line_interrupt(EXTI_line_t line, uint8_t state, uint8_t irq_priority) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(line);
    MCAL_UNUSED(state);
    MCAL_UNUSED(irq_priority);
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_set_gpio_interrupt(GPIO_pin_t gpio, uint8_t state, uint8_t irq_priority) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    EXTI_port_t port = 0;
    uint32_t line = 0;
    uint8_t pin = 0;
    IRQn_Type irqn = 0;
    uint8_t enable_count = 0;
    // Check parameters.
    if (gpio >= GPIO_PIN_LAST) {
       status = MCAL_ERROR;
       goto errors;
    }
    port = EXTI_PIN_MAPPING[gpio].port;
    line = EXTI_PIN_MAPPING[gpio].line;
    pin = EXTI_PIN_MAPPING[gpio].pin;
    // Check port.
    switch (EXTI_PIN_MAPPING[gpio].port) {
    case EXTI_PORT_A:
        // Link to corresponding IRQ handler.
        irqn = GPIOA_IRQn;
        // Update init counter.
        if (state == 0) {
            if (exti_ctx.gpioa_irq_enable_count > 0) {
                exti_ctx.gpioa_irq_enable_count--;
            }
        }
        else {
            exti_ctx.gpioa_irq_enable_count++;
        }
        enable_count = exti_ctx.gpioa_irq_enable_count;
        break;
    case EXTI_PORT_B:
        // Link to corresponding IRQ handler.
        irqn = GPIOB_IRQn;
        // Update init counter.
        if (state == 0) {
            if (exti_ctx.gpiob_irq_enable_count > 0) {
                exti_ctx.gpiob_irq_enable_count--;
            }
        }
        else {
            exti_ctx.gpiob_irq_enable_count++;
        }
        enable_count = exti_ctx.gpiob_irq_enable_count;
        break;
    default:
        status = MCAL_ERROR;
        goto errors;
    }
    // Set interrupt state.
    if (state == 0) {
        // Disable line.
        LL_EXTI_DisableIT(line);
        LL_PWR_DisableWakeUpPin(EXTI_PWR_PORT[port], (1 << pin));
        // Disable interrupt.
        if (enable_count == 0) {
            NVIC_DisableIRQ(irqn);
        }
    }
    else {
        // Enable line.
        LL_PWR_ClearWakeupSource(EXTI_PWR_PORT[port], (1 << pin));
        LL_PWR_EnableWakeUpPin(EXTI_PWR_PORT[port], (1 << pin));
        LL_EXTI_ClearFlag(line);
        LL_EXTI_EnableIT(line);
        // Enable interrupt.
        NVIC_SetPriority(irqn, irq_priority);
        NVIC_EnableIRQ(irqn);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_clear_line_flag(EXTI_line_t line) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32WL3x.
    MCAL_UNUSED(line);
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_clear_gpio_flag(GPIO_pin_t gpio) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameters.
    if (gpio >= GPIO_PIN_LAST) {
       status = MCAL_ERROR;
       goto errors;
    }
    LL_EXTI_ClearFlag(EXTI_PIN_MAPPING[gpio].line);
errors:
    return status;
}
