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
#include "stddef.h"
#include "stdint.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_system.h"

/*** EXTI local macros ***/

#define EXTI_GPIO_PINS_PER_PORT     16

/*** EXTI local structures ***/

/*******************************************************************/
typedef struct {
    uint32_t port;
    EXTI_line_t line;
} EXTI_def_t;

/*******************************************************************/
typedef struct {
    EXTI_gpio_irq_cb_t gpio_irq_callbacks[EXTI_GPIO_PINS_PER_PORT];
    uint8_t exti_0_1_enable_count;
    uint8_t exti_2_3_enable_count;
    uint8_t exti_4_15_enable_count;
    uint8_t exti_rtc_enable_count;
} EXTI_context_t;

/*** EXTI local global variables ***/

static const EXTI_def_t EXTI_PIN_MAPPING[GPIO_PIN_LAST] = {
    // Digital pins.
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_3 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_2 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_10 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_3 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_5 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_4 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_10 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_8 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_9 },
    { LL_SYSCFG_EXTI_PORTC, EXTI_LINE_GPIO_7 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_6 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_7 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_6 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_5 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_9 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_8 },
    // Analog pins.
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_0 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_1 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_4 },
    { LL_SYSCFG_EXTI_PORTB, EXTI_LINE_GPIO_0 },
    { LL_SYSCFG_EXTI_PORTC, EXTI_LINE_GPIO_1 },
    { LL_SYSCFG_EXTI_PORTC, EXTI_LINE_GPIO_0 },
    // Push button.
    { LL_SYSCFG_EXTI_PORTC, EXTI_LINE_GPIO_13 },
    // LEDs.
    { 0, EXTI_LINE_LAST },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_5 },
    { 0, EXTI_LINE_LAST },
    // Virtual COM port.
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_2 },
    { LL_SYSCFG_EXTI_PORTA, EXTI_LINE_GPIO_3 },
};

static const uint8_t EXTI_LL_TRIGGER[EXTI_TRIGGER_LAST] = {
    LL_EXTI_TRIGGER_NONE,
    LL_EXTI_TRIGGER_RISING,
    LL_EXTI_TRIGGER_FALLING,
    LL_EXTI_TRIGGER_RISING_FALLING,
};

static EXTI_context_t exti_ctx;

/*** EXTI local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI0_1_IRQHandler(void) {
    // Line 0.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0)) {
        if (exti_ctx.gpio_irq_callbacks[0] != NULL) {
            exti_ctx.gpio_irq_callbacks[0]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    }
    // Line 1.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1)) {
        if (exti_ctx.gpio_irq_callbacks[1] != NULL) {
            exti_ctx.gpio_irq_callbacks[1]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    }
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI2_3_IRQHandler(void) {
    // Line 2.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2)) {
        if (exti_ctx.gpio_irq_callbacks[2] != NULL) {
            exti_ctx.gpio_irq_callbacks[2]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    }
    // Line 3.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3)) {
        if (exti_ctx.gpio_irq_callbacks[3] != NULL) {
            exti_ctx.gpio_irq_callbacks[3]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    }
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) EXTI4_15_IRQHandler(void) {
    // Line 4.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4)) {
        if (exti_ctx.gpio_irq_callbacks[4] != NULL) {
            exti_ctx.gpio_irq_callbacks[4]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    }
    // Line 5.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5)) {
        if (exti_ctx.gpio_irq_callbacks[5] != NULL) {
            exti_ctx.gpio_irq_callbacks[5]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
    }
    // Line 6.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6)) {
        if (exti_ctx.gpio_irq_callbacks[6] != NULL) {
            exti_ctx.gpio_irq_callbacks[6]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
    }
    // Line 7.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7)) {
        if (exti_ctx.gpio_irq_callbacks[7] != NULL) {
            exti_ctx.gpio_irq_callbacks[7]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
    }
    // Line 8.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_8)) {
        if (exti_ctx.gpio_irq_callbacks[8] != NULL) {
            exti_ctx.gpio_irq_callbacks[8]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
    }
    // Line 9.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9)) {
        if (exti_ctx.gpio_irq_callbacks[9] != NULL) {
            exti_ctx.gpio_irq_callbacks[9]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
    }
    // Line 10.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10)) {
        if (exti_ctx.gpio_irq_callbacks[10] != NULL) {
            exti_ctx.gpio_irq_callbacks[10]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    }
    // Line 11.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11)) {
        if (exti_ctx.gpio_irq_callbacks[11] != NULL) {
            exti_ctx.gpio_irq_callbacks[11]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
    }
    // Line 12.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12)) {
        if (exti_ctx.gpio_irq_callbacks[12] != NULL) {
            exti_ctx.gpio_irq_callbacks[12]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    }
    // Line 13.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13)) {
        if (exti_ctx.gpio_irq_callbacks[13] != NULL) {
            exti_ctx.gpio_irq_callbacks[13]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    }
    // Line 14.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_14)) {
        if (exti_ctx.gpio_irq_callbacks[14] != NULL) {
            exti_ctx.gpio_irq_callbacks[14]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_14);
    }
    // Line 15.
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15)) {
        if (exti_ctx.gpio_irq_callbacks[15] != NULL) {
            exti_ctx.gpio_irq_callbacks[15]();
        }
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    }
}

/*** EXTI functions ***/

/*******************************************************************/
void EXTI_init(void) {
    // Local variables.
    uint8_t idx = 0;
    // Enable peripheral clock.
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    // Init context.
    for (idx = 0; idx < EXTI_GPIO_PINS_PER_PORT; idx++) {
        exti_ctx.gpio_irq_callbacks[idx] = NULL;
    }
    exti_ctx.exti_0_1_enable_count = 0;
    exti_ctx.exti_2_3_enable_count = 0;
    exti_ctx.exti_4_15_enable_count = 0;
    exti_ctx.exti_rtc_enable_count = 0;
}

/*******************************************************************/
void EXTI_de_init(void) {
    // Disable peripheral clock.
    LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
}


/*******************************************************************/
MCAL_status_t EXTI_configure_line(EXTI_line_t line, uint8_t state, EXTI_trigger_t trigger) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    LL_EXTI_InitTypeDef exti_init;
    // Check parameters.
    if ((line >= EXTI_LINE_LAST) || (trigger >= EXTI_TRIGGER_LAST)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Configure line.
    exti_init.Line_0_31 = (1 << line);
    exti_init.LineCommand = ((state != 0) ? ENABLE : DISABLE);
    exti_init.Mode = LL_EXTI_MODE_IT;
    exti_init.Trigger = ((state != 0) ? EXTI_LL_TRIGGER[trigger] : LL_EXTI_TRIGGER_NONE);
    ll_status = LL_EXTI_Init(&exti_init);
    _check_ll_status();
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_configure_gpio(GPIO_pin_t gpio, uint8_t state, EXTI_trigger_t trigger, EXTI_gpio_irq_cb_t irq_callback) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint8_t exti_line = 0;
    uint32_t syscfg_line = 0;
    // Check parameters.
    if (gpio >= GPIO_PIN_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    exti_line = EXTI_PIN_MAPPING[gpio].line;
    // Compute SYSCFG line.
    syscfg_line = ((exti_line % 4) << 18);
    syscfg_line |= (exti_line >> 2);
    // Init line.
    status = EXTI_configure_line(exti_line, state, trigger);
    if (status != MCAL_SUCCESS) goto errors;
    // Configure port and callback in case of GPIO (line 0 to 15).
    LL_SYSCFG_SetEXTISource(EXTI_PIN_MAPPING[gpio].port, syscfg_line);
    // Register callback.
    exti_ctx.gpio_irq_callbacks[exti_line] = irq_callback;
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_set_line_interrupt(EXTI_line_t line, uint8_t state, uint8_t irq_priority) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t exti_line = (0b1 << line);
    IRQn_Type irqn = 0;
    uint8_t enable_count = 0;
    // Enable corresponding line.
    switch (line) {
    case EXTI_LINE_GPIO_0:
    case EXTI_LINE_GPIO_1:
        // Link to corresponding IRQ handler.
        irqn = EXTI0_1_IRQn;
        // Update init counter.
        if (state == 0) {
            if (exti_ctx.exti_0_1_enable_count > 0) {
                exti_ctx.exti_0_1_enable_count--;
            }
        }
        else {
            exti_ctx.exti_0_1_enable_count++;
        }
        enable_count = exti_ctx.exti_0_1_enable_count;
        break;
    case EXTI_LINE_GPIO_2:
    case EXTI_LINE_GPIO_3:
        // Link to corresponding IRQ handler.
        irqn = EXTI2_3_IRQn;
        // Update init counter.
        if (state == 0) {
            if (exti_ctx.exti_2_3_enable_count > 0) {
                exti_ctx.exti_2_3_enable_count--;
            }
        }
        else {
            exti_ctx.exti_2_3_enable_count++;
        }
        enable_count = exti_ctx.exti_2_3_enable_count;
        break;
    case EXTI_LINE_GPIO_4:
    case EXTI_LINE_GPIO_5:
    case EXTI_LINE_GPIO_6:
    case EXTI_LINE_GPIO_7:
    case EXTI_LINE_GPIO_8:
    case EXTI_LINE_GPIO_9:
    case EXTI_LINE_GPIO_10:
    case EXTI_LINE_GPIO_11:
    case EXTI_LINE_GPIO_12:
    case EXTI_LINE_GPIO_13:
    case EXTI_LINE_GPIO_14:
    case EXTI_LINE_GPIO_15:
        // Link to corresponding IRQ handler.
        irqn = EXTI4_15_IRQn;
        // Update init counter.
        if (state == 0) {
            if (exti_ctx.exti_4_15_enable_count > 0) {
                exti_ctx.exti_4_15_enable_count--;
            }
        }
        else {
            exti_ctx.exti_4_15_enable_count++;
        }
        enable_count = exti_ctx.exti_4_15_enable_count;
        break;
    case EXTI_LINE_RTC_ALARM:
    case EXTI_LINE_RTC_TAMPER_TIMESTAMP:
    case EXTI_LINE_RTC_WAKEUP_TIMER:
        irqn = RTC_IRQn;
        exti_ctx.exti_rtc_enable_count++;
        break;
    case EXTI_LINE_I2C1:
        irqn = I2C1_IRQn;
        break;
    case EXTI_LINE_USART1:
        irqn = USART1_IRQn;
        break;
    case EXTI_LINE_USART2:
        irqn = USART2_IRQn;
        break;
    case EXTI_LINE_LPUART1:
        irqn = LPUART1_IRQn;
        break;
    case EXTI_LINE_LPTIM1:
        irqn = LPTIM1_IRQn;
        break;
    default:
        status = MCAL_ERROR;
        goto errors;
    }
    // Set interrupt state.
    if (state == 0) {
        // Disable line.
        LL_EXTI_DisableIT_0_31(exti_line);
        // Disable interrupt.
        if (enable_count == 0) {
            NVIC_DisableIRQ(irqn);
        }
    }
    else {
        // Enable line.
        LL_EXTI_ClearFlag_0_31(exti_line);
        LL_EXTI_EnableIT_0_31(exti_line);
        // Enable interrupt.
        NVIC_SetPriority(irqn, irq_priority);
        NVIC_EnableIRQ(irqn);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_set_gpio_interrupt(GPIO_pin_t gpio, uint8_t state, uint8_t irq_priority) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t exti_line = 0;
    // Check parameters.
    if (gpio >= GPIO_PIN_LAST) {
       status = MCAL_ERROR;
       goto errors;
    }
    exti_line = EXTI_PIN_MAPPING[gpio].line;
    // Set corresponding line.
    status = EXTI_set_line_interrupt(exti_line, state, irq_priority);
    if (status != MCAL_SUCCESS) goto errors;
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_clear_line_flag(EXTI_line_t line) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t exti_line = (0b1 << line);
    // Check parameter.
    if (line >= EXTI_LINE_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    LL_EXTI_ClearFlag_0_31(exti_line);
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t EXTI_clear_gpio_flag(GPIO_pin_t gpio) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t exti_line = 0;
    // Check parameters.
    if (gpio >= GPIO_PIN_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    exti_line = EXTI_PIN_MAPPING[gpio].line;
    // Clear corresponding line.
    status = EXTI_clear_line_flag(exti_line);
    if (status != MCAL_SUCCESS) goto errors;
errors:
    return status;
}
