/*!*****************************************************************
 * \file    lr1110mb1djs_hw_api.c
 * \brief   LR1110MB1DJS shield HW API implementation.
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

#include "board/lr11xx_hw_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
// Sigfox EP library.
#include "sigfox_error.h"
#include "sigfox_types.h"

#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "lr11xx_mapping.h"
#include "nvic.h"
#include "stdint.h"
#include "stddef.h"
#include "spi.h"

/*** LR1110MB1DJS HW API local macros ***/

#define LR1110MB1DJS_FREQ_MIN   150000000
#define LR1110MB1DJS_FREQ_MAX   960000000

#define LR1110MB1DJS_POWER_MIN  -17
#define LR1110MB1DJS_POWER_MAX  22

/***LR1110MB1DJS HW API global variables ***/

const LR11XX_MAPPING_gpios_t LR11XX_MAPPING_gpios = {
    .spi_sck = {GPIO_PORT_A, 5, 0},
    .spi_miso = {GPIO_PORT_A, 6, 0},
    .spi_mosi = {GPIO_PORT_A, 7, 0},
    .spi_nss = {GPIO_PORT_A, 8, 0},
    .busy = {GPIO_PORT_B, 3, 0},
    .irq = {GPIO_PORT_B, 4, 0},
    .reset = {GPIO_PORT_A, 0, 0},
    .lna = {GPIO_PORT_B, 0, 0},
    .led_tx = {GPIO_PORT_C, 1, 0},
    .led_rx = {GPIO_PORT_C, 0, 0},
    .led_scan = {GPIO_PORT_B, 5, 0},
};

/*** LR11XX HW API local functions ***/

const LR11XX_HW_API_pa_pwr_cfg_t _lr1110mb1djs_pa_pwr_cfg_table[LR1110MB1DJS_POWER_MAX - LR1110MB1DJS_POWER_MIN + 1] = {
    {
        // Expected output power = -17dBm
        .power = -15,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -16dBm
        .power = -14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -15dBm
        .power = -13,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -14dBm
        .power = -12,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -13dBm
        .power = -11,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -12dBm
        .power = -9,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -11dBm
        .power = -8,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -10dBm
        .power = -7,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -9dBm
        .power = -6,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -8dBm
        .power = -5,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -7dBm
        .power = -4,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -6dBm
        .power = -3,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -5dBm
        .power = -2,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -4dBm
        .power = -1,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -3dBm
        .power = 0,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -2dBm
        .power = 1,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = -1dBm
        .power = 2,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 0dBm
        .power = 3,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 1dBm
        .power = 3,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 2dBm
        .power = 4,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 3dBm
        .power = 7,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 4dBm
        .power = 8,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 5dBm
        .power = 9,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 6dBm
        .power = 10,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 7dBm
        .power = 12,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 8dBm
        .power = 13,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 9dBm
        .power = 14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 10dBm
        .power = 14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 11dBm
        .power = 13,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 12dBm
        .power = 14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 13dBm
        .power = 14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x04,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 14dBm
        .power = 14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x05,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 15dBm
        .power = 14,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_LP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VREG,
        .pa_config.pa_duty_cycle = 0x07,
        .pa_config.pa_hp_sel = 0x00,
    },
    {
        // Expected output power = 16dBm
        .power = 22,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x03,
        .pa_config.pa_hp_sel = 0x03,
    },
    {
        // Expected output power = 17dBm
        .power = 22,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x04,
        .pa_config.pa_hp_sel = 0x03,
    },
    {
        // Expected output power = 18dBm
        .power = 22,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.pa_hp_sel = 0x05,
    },
    {
        // Expected output power = 19dBm
        .power = 22,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x05,
        .pa_config.pa_hp_sel = 0x04,
    },
    {
        // Expected output power = 20dBm
        .power = 22,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x03,
        .pa_config.pa_hp_sel = 0x07,
    },
    {
        // Expected output power = 21dBm
        .power = 21,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x04,
        .pa_config.pa_hp_sel = 0x07,
    },
    {
        // Expected output power = 22dBm
        .power = 22,
        .pa_config.pa_sel = LR11XX_HW_API_RADIO_PA_SEL_HP,
        .pa_config.pa_reg_supply = LR11XX_HW_API_RADIO_PA_REG_SUPPLY_VBAT,
        .pa_config.pa_duty_cycle = 0x04,
        .pa_config.pa_hp_sel = 0x07,
    },
};

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) SIGFOX_EXIT_ERROR(LR11XX_HW_API_ERROR); }

/*** LR11XX HW API functions ***/

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_open(LR11XX_HW_API_config_t *hw_api_config) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    SPI_gpio_t spi_gpio;
    // Configure hardware interface.
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.reset, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.irq, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.busy, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_UP);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.led_scan, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.led_tx, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.led_rx, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    // Init SPI peripheral.
    spi_gpio.miso = &LR11XX_MAPPING_gpios.spi_miso;
    spi_gpio.mosi = &LR11XX_MAPPING_gpios.spi_mosi;
    spi_gpio.sck = &LR11XX_MAPPING_gpios.spi_sck;
    mcal_status = SPI_init(&spi_gpio);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.spi_nss, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_write(&LR11XX_MAPPING_gpios.spi_nss, 1);
    _check_mcal_status();
    // Init IRQ line.
    mcal_status = EXTI_configure(LR11XX_GPIO_IRQ_EXTI_PORT, LR11XX_GPIO_IRQ_EXTI_LINE, EXTI_TRIGGER_RISING, hw_api_config->gpio_irq_callback);
    _check_mcal_status();
    mcal_status = EXTI_enable_irq(LR11XX_GPIO_IRQ_EXTI_LINE, NVIC_IRQ_PRIORITY_EXTI_RADIO);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_close(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    SPI_gpio_t spi_gpio;
    // Disable EXTI line.
    mcal_status = EXTI_de_configure(LR11XX_GPIO_IRQ_EXTI_LINE);
    _check_mcal_status();
    // Release SPI peripheral.
    spi_gpio.miso = &LR11XX_MAPPING_gpios.spi_miso;
    spi_gpio.mosi = &LR11XX_MAPPING_gpios.spi_mosi;
    spi_gpio.sck = &LR11XX_MAPPING_gpios.spi_sck;
    mcal_status = SPI_de_init(&spi_gpio);
    _check_mcal_status();
    // Put GPIOs in high impedance.
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.spi_nss, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.led_scan, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.reset, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.irq, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.busy, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.led_tx, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&LR11XX_MAPPING_gpios.led_rx, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_delayMs(unsigned short delay_ms) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Use LPTIM.
    mcal_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_fem_mask(LR11XX_HW_API_FEM_t fem, sfx_u8 *rfsw_dio_mask) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    switch (fem) {
    case LR11XX_HW_API_FEM_PIN_USED:
        (*rfsw_dio_mask) = LR11XX_HW_API_RFSW0_DIO5 | LR11XX_HW_API_RFSW1_DIO6 | LR11XX_HW_API_RFSW2_DIO7;
        break;
    case LR11XX_HW_API_FEM_STBY:
        (*rfsw_dio_mask) = 0x00;
        break;
    case LR11XX_HW_API_FEM_RX:
        (*rfsw_dio_mask) = LR11XX_HW_API_RFSW0_DIO5;
        break;
    case LR11XX_HW_API_FEM_TX:
        (*rfsw_dio_mask) = LR11XX_HW_API_RFSW0_DIO5 | LR11XX_HW_API_RFSW1_DIO6;
        break;
    case LR11XX_HW_API_FEM_TXHP:
        (*rfsw_dio_mask) = LR11XX_HW_API_RFSW1_DIO6;
        break;
    case LR11XX_HW_API_FEM_WIFI:
        (*rfsw_dio_mask) = 0x00;
        break;
    case LR11XX_HW_API_FEM_GNSS:
        (*rfsw_dio_mask) = LR11XX_HW_API_RFSW2_DIO7;
        break;
    default:
        (*rfsw_dio_mask) = 0x00;
        break;
    }
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_xosc_cfg(LR11XX_HW_API_xosc_cfg_t *xosc_cfg) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    // Update oscillator configuration.
    xosc_cfg->has_tcxo = 0x01;
    xosc_cfg->tcxo_supply_voltage = LR11XX_HW_API_TCXO_CTRL_3_0V;
    xosc_cfg->startup_time_in_tick = 300;
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_pa_pwr_cfg(LR11XX_HW_API_pa_pwr_cfg_t *pa_pwr_cfg, sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    // Check frequency.
    if ((rf_freq_in_hz < LR1110MB1DJS_FREQ_MIN) || (rf_freq_in_hz > LR1110MB1DJS_FREQ_MAX)) {
        SIGFOX_EXIT_ERROR(LR11XX_HW_API_ERROR);
    }
    // Check power.
    if ((expected_output_pwr_in_dbm < LR1110MB1DJS_POWER_MIN) || (expected_output_pwr_in_dbm > LR1110MB1DJS_POWER_MAX)) {
        SIGFOX_EXIT_ERROR(LR11XX_HW_API_ERROR);
    }
    // Update PA configuration.
    pa_pwr_cfg->power = _lr1110mb1djs_pa_pwr_cfg_table[expected_output_pwr_in_dbm - LR1110MB1DJS_POWER_MIN].power;
    pa_pwr_cfg->pa_config.pa_sel = _lr1110mb1djs_pa_pwr_cfg_table[expected_output_pwr_in_dbm - LR1110MB1DJS_POWER_MIN].pa_config.pa_sel;
    pa_pwr_cfg->pa_config.pa_reg_supply = _lr1110mb1djs_pa_pwr_cfg_table[expected_output_pwr_in_dbm - LR1110MB1DJS_POWER_MIN].pa_config.pa_reg_supply;
    pa_pwr_cfg->pa_config.pa_duty_cycle = _lr1110mb1djs_pa_pwr_cfg_table[expected_output_pwr_in_dbm - LR1110MB1DJS_POWER_MIN].pa_config.pa_duty_cycle;
    pa_pwr_cfg->pa_config.pa_hp_sel = _lr1110mb1djs_pa_pwr_cfg_table[expected_output_pwr_in_dbm - LR1110MB1DJS_POWER_MIN].pa_config.pa_hp_sel;
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_tx_on(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn TX LED on.
    if (LR11XX_MAPPING_gpios.led_tx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&LR11XX_MAPPING_gpios.led_tx, 1);
    }
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_tx_off(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn TX LED off.
    if (LR11XX_MAPPING_gpios.led_tx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&LR11XX_MAPPING_gpios.led_tx, 0);
    }
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_rx_on(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn RX LED on.
    if (LR11XX_MAPPING_gpios.led_rx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&LR11XX_MAPPING_gpios.led_rx, 1);
    }
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_rx_off(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn RX LED off.
    if (LR11XX_MAPPING_gpios.led_rx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&LR11XX_MAPPING_gpios.led_rx, 0);
    }
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
LR11XX_HW_API_status_t LR11XX_HW_API_get_latency(LR11XX_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    LR11XX_HW_API_status_t status = LR11XX_HW_API_SUCCESS;
#endif
    // Check parameter.
    if (latency_ms == NULL) {
        SIGFOX_EXIT_ERROR(LR11XX_HW_API_ERROR);
    }
    // Update latency value.
    switch (latency_type) {
    case LR11XX_HW_API_LATENCY_RESET:
        (*latency_ms) = LR11XX_HAL_RESET_DELAY_MS;
        break;
    case LR11XX_HW_API_LATENCY_WAKEUP:
        (*latency_ms) = LR11XX_HAL_WAKEUP_DELAY_MS;
        break;
    default:
        SIGFOX_EXIT_ERROR(LR11XX_HW_API_ERROR);
        break;
    }
errors:
    SIGFOX_RETURN();
}
#endif
