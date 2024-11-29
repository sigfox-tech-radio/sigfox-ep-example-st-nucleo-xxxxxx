/*!*****************************************************************
 * \file    tim.c
 * \brief   Common TIM driver based on LL driver.
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

#include "tim.h"

#include "mcal.h"
#include "pwr.h"
#include "rcc.h"
#include "stddef.h"
#include "stdint.h"
#ifdef STM32L0XX
#include "stm32l0xx_hal_conf.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_tim.h"
#include "stm32l0xx_ll_rcc.h"
#endif

/*** TIM local macros ***/

#define TIM_INSTANCE                    TIM2
#define TIM_NVIC_IRQN                   TIM2_IRQn
#define TIM_REMAP_LSE                   LL_TIM_TIM2_ETR_RMP_LSE

#define TIM_CNT_VALUE_MAX               0xFFFF

#define TIM_TARGET_TRIGGER_CLOCK_HZ     2048

#define TIM_PRESCALER_ETRF_LSE          1
#define TIM_PRESCALER_PSC_LSE           ((trigger_clock_hz) / (TIM_TARGET_TRIGGER_CLOCK_HZ * TIM_PRESCALER_ETRF_LSE))

#define TIM_CLOCK_SWITCH_LATENCY_MS     2

#define TIM_TIMER_DURATION_MS_MIN       1
#define TIM_TIMER_DURATION_MS_MAX       ((TIM_CNT_VALUE_MAX * 1000) / (tim_ctx.etrf_clock_hz))

/*** TIM local structures ***/

/*******************************************************************/
typedef void (*LL_TIM_OC_SetCompare)(TIM_TypeDef *TIMx, uint32_t CompareValue);
typedef void (*LL_TIM_ClearFlag)(TIM_TypeDef *TIMx);
typedef void (*LL_TIM_EnableIT)(TIM_TypeDef *TIMx);
typedef void (*LL_TIM_DisableIT)(TIM_TypeDef *TIMx);
typedef uint32_t (*LL_TIM_IsActiveFlag)(const TIM_TypeDef *TIMx);

/*******************************************************************/
typedef struct {
    uint32_t duration_ms;
    TIM_waiting_mode_t waiting_mode;
    volatile uint8_t running_flag;
    volatile uint8_t irq_flag;
    TIM_completion_irq_cb_t irq_callback;
    LL_TIM_OC_SetCompare set_compare_pfn;
    LL_TIM_EnableIT enable_it_pfn;
    LL_TIM_DisableIT disable_it_pfn;
    LL_TIM_ClearFlag clear_flag_pfn;
    LL_TIM_IsActiveFlag is_active_flag_pfn;
} TIM_channel_context_t;

/*******************************************************************/
typedef struct {
    uint32_t etrf_clock_hz;
    TIM_channel_context_t channel[TIM_CHANNEL_LAST];
    uint8_t channel_running_mask;
} TIM_context_t;

/*** TIM local global variables ***/

static TIM_context_t tim_ctx;

/*** TIM local functions ***/

/*******************************************************************/
#define _check_ll_status(void) { if (ll_status != SUCCESS) { status = MCAL_ERROR; goto errors; } }

/*******************************************************************/
static void _TIM_compute_compare_value(TIM_channel_t channel) {
    // Local variables.
    uint32_t cnt_irq = 0;
    // Compute next period value.
    cnt_irq = (LL_TIM_GetCounter(TIM_INSTANCE) + ((tim_ctx.channel[channel].duration_ms * tim_ctx.etrf_clock_hz) / (1000))) % TIM_CNT_VALUE_MAX;
    tim_ctx.channel[channel].set_compare_pfn(TIM_INSTANCE, cnt_irq);
}

/*******************************************************************/
void __attribute__((optimize("-O0"))) TIM2_IRQHandler(void) {
    // Local variables.
    uint8_t channel_idx = 0;
    // Channels loop.
    for (channel_idx = 0; channel_idx < TIM_CHANNEL_LAST; channel_idx++) {
        // Check flag.
        if (tim_ctx.channel[channel_idx].is_active_flag_pfn(TIM_INSTANCE) != 0) {
            // Check if channel is active.
            if (tim_ctx.channel[channel_idx].running_flag != 0) {
                // Set local flag.
                tim_ctx.channel[channel_idx].irq_flag = tim_ctx.channel[channel_idx].running_flag;
                // Compute next compare value.
                _TIM_compute_compare_value(channel_idx);
                // Call callback.
                if (tim_ctx.channel[channel_idx].irq_callback != NULL) {
                    tim_ctx.channel[channel_idx].irq_callback();
                }
            }
            // Clear flag.
            tim_ctx.channel[channel_idx].clear_flag_pfn(TIM_INSTANCE);
        }
    }
}

/*** TIM functions ***/

/*******************************************************************/
MCAL_status_t TIM_init(uint8_t irq_priority) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    ErrorStatus ll_status = SUCCESS;
    LL_TIM_InitTypeDef tim_init;
    LL_TIM_OC_InitTypeDef tim_oc_init;
    uint32_t trigger_clock_hz = LSE_VALUE;
    uint8_t idx = 0;
    // Enable peripheral clock.
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_APB1_GRP1_EnableClockSleep(LL_APB1_GRP1_PERIPH_TIM2);
    // Init context.
    tim_ctx.etrf_clock_hz = ((trigger_clock_hz) / (TIM_PRESCALER_ETRF_LSE * TIM_PRESCALER_PSC_LSE));
    tim_ctx.channel_running_mask = 0;
    // Init common channels parameters.
    for (idx = 0; idx < TIM_CHANNEL_LAST; idx++) {
        tim_ctx.channel[idx].duration_ms = 0;
        tim_ctx.channel[idx].irq_callback = NULL;
    }
    // Init CH1.
    tim_ctx.channel[TIM_CHANNEL_CH1].set_compare_pfn = &LL_TIM_OC_SetCompareCH1;
    tim_ctx.channel[TIM_CHANNEL_CH1].enable_it_pfn = &LL_TIM_EnableIT_CC1;
    tim_ctx.channel[TIM_CHANNEL_CH1].disable_it_pfn = &LL_TIM_DisableIT_CC1;
    tim_ctx.channel[TIM_CHANNEL_CH1].is_active_flag_pfn = &LL_TIM_IsActiveFlag_CC1;
    tim_ctx.channel[TIM_CHANNEL_CH1].clear_flag_pfn = &LL_TIM_ClearFlag_CC1;
    // Init CH2.
    tim_ctx.channel[TIM_CHANNEL_CH2].set_compare_pfn = &LL_TIM_OC_SetCompareCH2;
    tim_ctx.channel[TIM_CHANNEL_CH2].enable_it_pfn = &LL_TIM_EnableIT_CC2;
    tim_ctx.channel[TIM_CHANNEL_CH2].disable_it_pfn = &LL_TIM_DisableIT_CC2;
    tim_ctx.channel[TIM_CHANNEL_CH2].is_active_flag_pfn = &LL_TIM_IsActiveFlag_CC2;
    tim_ctx.channel[TIM_CHANNEL_CH2].clear_flag_pfn = &LL_TIM_ClearFlag_CC2;
    // Init CH3.
    tim_ctx.channel[TIM_CHANNEL_CH3].set_compare_pfn = &LL_TIM_OC_SetCompareCH3;
    tim_ctx.channel[TIM_CHANNEL_CH3].enable_it_pfn = &LL_TIM_EnableIT_CC3;
    tim_ctx.channel[TIM_CHANNEL_CH3].disable_it_pfn = &LL_TIM_DisableIT_CC3;
    tim_ctx.channel[TIM_CHANNEL_CH3].is_active_flag_pfn = &LL_TIM_IsActiveFlag_CC3;
    tim_ctx.channel[TIM_CHANNEL_CH3].clear_flag_pfn = &LL_TIM_ClearFlag_CC3;
    // Init CH4.
    tim_ctx.channel[TIM_CHANNEL_CH4].set_compare_pfn = &LL_TIM_OC_SetCompareCH4;
    tim_ctx.channel[TIM_CHANNEL_CH4].enable_it_pfn = &LL_TIM_EnableIT_CC4;
    tim_ctx.channel[TIM_CHANNEL_CH4].disable_it_pfn = &LL_TIM_DisableIT_CC4;
    tim_ctx.channel[TIM_CHANNEL_CH4].is_active_flag_pfn = &LL_TIM_IsActiveFlag_CC4;
    tim_ctx.channel[TIM_CHANNEL_CH4].clear_flag_pfn = &LL_TIM_ClearFlag_CC4;
    // Use LSE as trigger.
    LL_TIM_ConfigETR(TIM_INSTANCE, LL_TIM_ETR_POLARITY_NONINVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV1);
    LL_TIM_SetRemap(TIM_INSTANCE, TIM_REMAP_LSE);
    LL_TIM_EnableExternalClock(TIM_INSTANCE);
    LL_TIM_SetTriggerInput(TIM_INSTANCE, LL_TIM_TS_ETRF);
    // Configure channels in output compare mode.
    tim_oc_init.OCMode = LL_TIM_OCMODE_FROZEN;
    tim_oc_init.OCState = LL_TIM_OCSTATE_DISABLE;
    tim_oc_init.CompareValue = 0;
    tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    ll_status = LL_TIM_OC_Init(TIM_INSTANCE, LL_TIM_CHANNEL_CH1, &tim_oc_init);
    _check_ll_status();
    ll_status = LL_TIM_OC_Init(TIM_INSTANCE, LL_TIM_CHANNEL_CH2, &tim_oc_init);
    _check_ll_status();
    ll_status = LL_TIM_OC_Init(TIM_INSTANCE, LL_TIM_CHANNEL_CH3, &tim_oc_init);
    _check_ll_status();
    ll_status = LL_TIM_OC_Init(TIM_INSTANCE, LL_TIM_CHANNEL_CH4, &tim_oc_init);
    _check_ll_status();
    // Init timer.
    tim_init.Prescaler = (TIM_PRESCALER_PSC_LSE - 1);
    tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
    tim_init.Autoreload = TIM_CNT_VALUE_MAX;
    tim_init.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    ll_status = LL_TIM_Init(TIM_INSTANCE, &tim_init);
    _check_ll_status();
    // Configure timer interrupt.
    NVIC_SetPriority(TIM_NVIC_IRQN, irq_priority);
    NVIC_EnableIRQ(TIM_NVIC_IRQN);
    // Generate event to update registers.
    LL_TIM_GenerateEvent_UPDATE(TIM_INSTANCE);
errors:
    return status;
}

/*******************************************************************/
void TIM_de_init(void) {
    // Disable all channels.
    LL_TIM_CC_DisableChannel(TIM_INSTANCE, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM_INSTANCE, LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM_INSTANCE, LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM_INSTANCE, LL_TIM_CHANNEL_CH4);
    // Disable peripheral.
    LL_TIM_DisableCounter(TIM_INSTANCE);
    // Disable clock.
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_TIM2);
}

/*******************************************************************/
MCAL_status_t TIM_start(TIM_channel_t channel, uint32_t duration_ms, TIM_waiting_mode_t waiting_mode, TIM_completion_irq_cb_t irq_callback) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint32_t local_duration_ms = duration_ms;
    uint32_t duration_min_ms = TIM_TIMER_DURATION_MS_MIN;
    // Check waiting mode.
    if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
        // Compensate clock switch latency.
        duration_min_ms += TIM_CLOCK_SWITCH_LATENCY_MS;
    }
    // Check parameters.
    if ((channel >= TIM_CHANNEL_LAST) || (waiting_mode >= TIM_WAITING_MODE_LAST) || (duration_ms < duration_min_ms) || (duration_ms > TIM_TIMER_DURATION_MS_MAX)) {
        status = MCAL_ERROR;
        goto errors;
    }
    if (waiting_mode == TIM_WAITING_MODE_LOW_POWER_SLEEP) {
        local_duration_ms -= TIM_CLOCK_SWITCH_LATENCY_MS;
    }
    // Update channel context.
    tim_ctx.channel[channel].duration_ms = local_duration_ms;
    tim_ctx.channel[channel].waiting_mode = waiting_mode;
    tim_ctx.channel[channel].irq_callback = irq_callback;
    tim_ctx.channel[channel].running_flag = 1;
    tim_ctx.channel[channel].irq_flag = 0;
    // Update running mask.
    tim_ctx.channel_running_mask |= (0b1 << channel);
    // Compute compare value.
    _TIM_compute_compare_value(channel);
    // Clear flag.
    tim_ctx.channel[channel].clear_flag_pfn(TIM_INSTANCE);
    // Enable channel.
    tim_ctx.channel[channel].enable_it_pfn(TIM_INSTANCE);
    // Start timer.
    LL_TIM_EnableCounter(TIM_INSTANCE);
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t TIM_stop(TIM_channel_t channel) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameter.
    if (channel >= TIM_CHANNEL_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Disable channel.
    tim_ctx.channel[channel].disable_it_pfn(TIM_INSTANCE);
    // Clear flags.
    tim_ctx.channel[channel].clear_flag_pfn(TIM_INSTANCE);
    tim_ctx.channel[channel].running_flag = 0;
    tim_ctx.channel[channel].irq_flag = 0;
    // Update running mask.
    tim_ctx.channel_running_mask &= ~(0b1 << channel);
    // Disable counter if all channels are stopped.
    if (tim_ctx.channel_running_mask == 0) {
        LL_TIM_DisableCounter(TIM_INSTANCE);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t TIM_get_status(TIM_channel_t channel, uint8_t *timer_has_elapsed) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameters.
    if ((channel >= TIM_CHANNEL_LAST) || (timer_has_elapsed == NULL)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Update flag.
    (*timer_has_elapsed) = ((tim_ctx.channel[channel].running_flag == 0) || (tim_ctx.channel[channel].irq_flag != 0)) ? 1 : 0;
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t TIM_wait_completion(TIM_channel_t channel) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameters.
    if (channel >= TIM_CHANNEL_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Directly exit if the IRQ already occurred.
    if ((tim_ctx.channel[channel].running_flag == 0) || (tim_ctx.channel[channel].irq_flag != 0)) goto errors;
    // Sleep until channel is not running.
    switch (tim_ctx.channel[channel].waiting_mode) {
    case TIM_WAITING_MODE_ACTIVE:
        // Active loop.
        while (tim_ctx.channel[channel].irq_flag == 0);
        break;
    case TIM_WAITING_MODE_SLEEP:
        // Enter sleep mode.
        while (tim_ctx.channel[channel].irq_flag == 0) {
            PWR_enter_sleep_mode();
        }
        break;
    case TIM_WAITING_MODE_LOW_POWER_SLEEP:
        // Switch to MSI.
        status = RCC_switch_to_msi(RCC_MSI_RANGE_1_131KHZ);
        if (status != MCAL_SUCCESS) goto errors;
        // Enter low power sleep mode.
        while (tim_ctx.channel[channel].irq_flag == 0) {
            PWR_enter_low_power_sleep_mode();
        }
        // Go back to HSI.
        status = RCC_switch_to_hsi();
        if (status != MCAL_SUCCESS) goto errors;
        break;
    default:
        status = MCAL_ERROR;
        goto errors;
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t TIM_clear_irq_flag(TIM_channel_t channel) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    // Check parameters.
    if (channel >= TIM_CHANNEL_LAST) {
        status = MCAL_ERROR;
        goto errors;
    }
    tim_ctx.channel[channel].irq_flag = 0;
errors:
    return status;
}
