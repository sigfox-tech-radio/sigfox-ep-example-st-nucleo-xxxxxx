/*!*****************************************************************
 * \file    pwr.c
 * \brief   Common PWR driver based on LL driver.
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

#include "pwr.h"

#include "mcal.h"
#include "stdint.h"
#include "cmsis_gcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_system.h"

/*******************************************************************/
void PWR_init(void) {

    // Enable power interface clock.
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    // Unlock back-up registers (DBP bit).
    LL_PWR_EnableBkUpAccess();
    // Power memories down when entering sleep mode.
    LL_FLASH_EnableSleepPowerDown();
    // Use HSI clock when waking-up from stop mode.
    LL_RCC_SetClkAfterWakeFromStop(LL_RCC_STOP_WAKEUPCLOCK_HSI);
    // Switch internal voltage reference off in low power mode and ignore startup time.
    LL_PWR_EnableUltraLowPower();
    LL_PWR_EnableFastWakeUp();
    // Never return in low power sleep mode after wake-up.
    LL_LPM_DisableSleepOnExit();
#ifdef DEBUG
    // Enable debug MCU interface clock.
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_DBGMCU);
    // Enable debug in standby mode.
    LL_DBGMCU_EnableDBGStandbyMode();
    // Enable debug in sleep mode.
    LL_DBGMCU_EnableDBGSleepMode();
    // Enable debug in stop mode.
    LL_DBGMCU_EnableDBGStopMode();
#endif
}

/*******************************************************************/
void PWR_enter_sleep_mode(void) {
    // Regulator in normal mode.
    LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_MAIN);
    // Enter low power sleep mode.
    LL_LPM_EnableSleep();
    __WFI(); // Wait For Interrupt core instruction.
}

/*******************************************************************/
void PWR_enter_low_power_sleep_mode(void) {
    // Regulator in low power mode.
    LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);
    // Enter low power sleep mode.
    LL_LPM_EnableSleep();
    __WFI(); // Wait For Interrupt core instruction.
}

/*******************************************************************/
void PWR_enter_stop_mode(void) {
    // Regulator in low power mode.
    LL_PWR_SetRegulModeLP(LL_PWR_REGU_LPMODES_LOW_POWER);
    // Clear WUF flag.
    LL_PWR_ClearFlag_WU();
    // Enter stop mode when CPU enters deepsleep.
    LL_PWR_SetPowerMode(LL_PWR_MODE_STOP);
    // Enter stop mode.
    LL_LPM_EnableDeepSleep();
    __WFI(); // Wait For Interrupt core instruction.
}

/*******************************************************************/
MCAL_status_t PWR_set_smps_voltage(PWR_smps_voltage_t smps_voltage) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32L0xx.
    MCAL_UNUSED(smps_voltage);
    return status;
}

/*******************************************************************/
MCAL_status_t PWR_get_smps_voltage(PWR_smps_voltage_t *smps_voltage) {
    // Local variables.
    MCAL_status_t status = MCAL_ERROR;
    // Not used on STM32L0xx.
    MCAL_UNUSED(smps_voltage);
    return status;
}
