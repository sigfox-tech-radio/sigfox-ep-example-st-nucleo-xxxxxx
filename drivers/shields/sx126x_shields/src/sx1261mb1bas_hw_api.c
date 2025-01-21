/*!*****************************************************************
 * \file    sx1261mb1bas_hw_api.c
 * \brief   SX1261MB1BAS shield HW API implementation.
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

#include "board/sx126x_hw_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_error.h"
#include "sigfox_rc.h"

#include "exti.h"
#include "gpio.h"
#include "lptim.h"
#include "nvic.h"
#include "stdint.h"
#include "stddef.h"
#include "spi.h"
#include "sx126x_mapping.h"

#if (defined SIGFOX_EP_RC1_ZONE) || (defined SIGFOX_EP_RC6_ZONE) || (defined SIGFOX_EP_RC7_ZONE)

#define SX1261MB1BAS_FREQ_MIN 863000000
#define SX1261MB1BAS_FREQ_MAX 870000000

/***SX1261MB1BAS SX126X HW API local global variables ***/

const SX126X_MAPPING_gpios_t SX126X_MAPPING_gpios = {
    .spi_sck = {GPIO_PORT_A, 5, 0},
    .spi_miso = {GPIO_PORT_A, 6, 0},
    .spi_mosi = {GPIO_PORT_A, 7, 0},
    .spi_nss = {GPIO_PORT_A, 8, 0},
    .busy = {GPIO_PORT_B, 3, 0},
    .irq = {GPIO_PORT_B, 4, 0},
    .reset = {GPIO_PORT_A, 0, 0},
    .antenna_sw = {GPIO_PORT_A, 9, 0},
    .led_tx = {GPIO_PORT_C, 1, 0},
    .led_rx = {GPIO_PORT_C, 0, 0},
};

static const SX126X_HW_API_pa_pwr_cfg_t _sx1261mb1bas_pa_pwr_cfg_table[SX126X_SHIELDS_SX1261_MAX_PWR - SX126X_SHIELDS_SX1261_MIN_PWR + 1] = {
    {
        // Expected output power = -17dBm
        .power = -15,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -16dBm
        .power = -14,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -15dBm
        .power = -15,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x03,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -14dBm
        .power = -11,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -13dBm
        .power = -13,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x03,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -12dBm
        .power = -9,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -11dBm
        .power = -8,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -10dBm
        .power = -7,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -9dBm
        .power = -8,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -8dBm
        .power = -7,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -7dBm
        .power = -6,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -6dBm
        .power = -4,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -5dBm
        .power = -3,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -4dBm
        .power = -2,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -3dBm
        .power = -1,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -2dBm
        .power = 1,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = -1dBm
        .power = 1,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 0dBm
        .power = 2,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 1dBm
        .power = 5,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 2dBm
        .power = 6,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 3dBm
        .power = 7,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 4dBm
        .power = 8,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 5dBm
        .power = 8,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 6dBm
        .power = 9,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 7dBm
        .power = 12,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 8dBm
        .power = 13,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x00,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 9dBm
        .power = 13,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 10dBm
        .power = 14,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x01,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 11dBm
        .power = 14,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x02,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 12dBm
        .power = 12,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x05,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 13dBm
        .power = 14,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x03,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 14dBm
        .power = 14,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x04,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
    {
        // Expected output power = 15dBm
        .power = 14,
        .pa_config.hp_max = 0x00,
        .pa_config.pa_duty_cycle = 0x07,
        .pa_config.device_sel = 0x01,
        .pa_config.pa_lut = 0x01,
    },
};

/*** SX126X HW API local functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR); }

/*** SX126X HW API functions ***/

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_open(SX126X_HW_API_config_t *hw_api_config) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    SPI_gpio_t spi_gpio;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check input parameters.
    if (hw_api_config == NULL) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Configure hardware interface.
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.antenna_sw, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.antenna_sw, 1);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.reset, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.irq, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.busy, GPIO_MODE_INPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_UP);
    _check_mcal_status();
    if (SX126X_MAPPING_gpios.led_tx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.led_tx, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        _check_mcal_status();
    }
    if (SX126X_MAPPING_gpios.led_rx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.led_rx, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        _check_mcal_status();
    }
    // Init SPI peripheral.
    spi_gpio.miso = &SX126X_MAPPING_gpios.spi_miso;
    spi_gpio.mosi = &SX126X_MAPPING_gpios.spi_mosi;
    spi_gpio.sck = &SX126X_MAPPING_gpios.spi_sck;
    mcal_status = SPI_init(&spi_gpio);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.spi_nss, GPIO_MODE_OUTPUT, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_write(&SX126X_MAPPING_gpios.spi_nss, 1);
    _check_mcal_status();
    // Init IRQ line.
    mcal_status = EXTI_configure(SX126X_GPIO_IRQ_EXTI_PORT, SX126X_GPIO_IRQ_EXTI_LINE, EXTI_TRIGGER_RISING, hw_api_config->gpio_irq_callback);
    _check_mcal_status();
    mcal_status = EXTI_enable_irq(SX126X_GPIO_IRQ_EXTI_LINE, NVIC_IRQ_PRIORITY_EXTI_RADIO);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_close(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    SPI_gpio_t spi_gpio;
    // Disable EXTI line.
    mcal_status = EXTI_de_configure(SX126X_GPIO_IRQ_EXTI_LINE);
    _check_mcal_status();
    // Release SPI peripheral.
    spi_gpio.miso = &SX126X_MAPPING_gpios.spi_miso;
    spi_gpio.mosi = &SX126X_MAPPING_gpios.spi_mosi;
    spi_gpio.sck = &SX126X_MAPPING_gpios.spi_sck;
    mcal_status = SPI_de_init(&spi_gpio);
    _check_mcal_status();
    // Put GPIOs in high impedance.
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.spi_nss, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.antenna_sw, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.reset, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.irq, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.busy, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    _check_mcal_status();
    if (SX126X_MAPPING_gpios.led_tx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.led_tx, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        _check_mcal_status();
    }
    if (SX126X_MAPPING_gpios.led_rx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_configure(&SX126X_MAPPING_gpios.led_rx, GPIO_MODE_ANALOG, GPIO_OUTPUT_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
        _check_mcal_status();
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_delayMs(unsigned short delay_ms) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Use LPTIM.
    mcal_status = LPTIM_delay_milliseconds(delay_ms, LPTIM_DELAY_MODE_SLEEP);
    _check_mcal_status();
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_chip_name(SX126X_HW_API_chip_name_t *chipset) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (chipset == NULL) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Set chipset.
    (*chipset) = SX126X_HW_API_CHIP_NAME_SX1261;
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_reg_mode(SX126X_HW_API_reg_mod_t *reg_mode) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (reg_mode == NULL) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Set regulator mode.
    (*reg_mode) = SX126X_HW_API_REG_MODE_DCDC;
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_xosc_cfg(SX126X_HW_API_xosc_cfg_t *xosc_cfg) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (xosc_cfg == NULL) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    xosc_cfg->tcxo_is_radio_controlled = SIGFOX_FALSE;
    xosc_cfg->supply_voltage = SX126X_HW_API_TCXO_CTRL_3_0V;
    xosc_cfg->startup_time_in_tick = 300;
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_get_pa_pwr_cfg(SX126X_HW_API_pa_pwr_cfg_t *pa_pwr_cfg, sfx_u32 rf_freq_in_hz, sfx_s8 expected_output_pwr_in_dbm) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (pa_pwr_cfg == NULL) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
    if ((rf_freq_in_hz < SX1261MB1BAS_FREQ_MIN) || (rf_freq_in_hz > SX1261MB1BAS_FREQ_MAX)) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Check expected output power to avoid seg fault.
    if ((expected_output_pwr_in_dbm < SX126X_SHIELDS_SX1261_MIN_PWR) || (expected_output_pwr_in_dbm > SX126X_SHIELDS_SX1261_MAX_PWR)) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
    // Set PA configuration.
    pa_pwr_cfg->power = _sx1261mb1bas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SX126X_SHIELDS_SX1261_MIN_PWR].power;
    pa_pwr_cfg->pa_config.hp_max = _sx1261mb1bas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SX126X_SHIELDS_SX1261_MIN_PWR].pa_config.hp_max;
    pa_pwr_cfg->pa_config.pa_duty_cycle = _sx1261mb1bas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SX126X_SHIELDS_SX1261_MIN_PWR].pa_config.pa_duty_cycle;
    pa_pwr_cfg->pa_config.device_sel = _sx1261mb1bas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SX126X_SHIELDS_SX1261_MIN_PWR].pa_config.device_sel;
    pa_pwr_cfg->pa_config.pa_lut = _sx1261mb1bas_pa_pwr_cfg_table[expected_output_pwr_in_dbm - SX126X_SHIELDS_SX1261_MIN_PWR].pa_config.pa_lut;

errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_tx_on(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn TX LED on.
    if (SX126X_MAPPING_gpios.led_tx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&SX126X_MAPPING_gpios.led_tx, 1);
        _check_mcal_status();
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_tx_off(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn TX LED off.
    if (SX126X_MAPPING_gpios.led_tx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&SX126X_MAPPING_gpios.led_tx, 0);
        _check_mcal_status();
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_rx_on(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn RX LED on.
    if (SX126X_MAPPING_gpios.led_rx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&SX126X_MAPPING_gpios.led_rx, 1);
        _check_mcal_status();
    }

errors:
    SIGFOX_RETURN();
}
/*******************************************************************/
SX126X_HW_API_status_t SX126X_HW_API_rx_off(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
    MCAL_status_t mcal_status = MCAL_SUCCESS;
    // Turn RX LED off.
    if (SX126X_MAPPING_gpios.led_rx.port != GPIO_PORT_LAST) {
        mcal_status = GPIO_write(&SX126X_MAPPING_gpios.led_rx, 0);
        _check_mcal_status();
    }
errors:
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
SX126X_HW_API_status_t SX126X_HW_API_get_latency(SX126X_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SX126X_HW_API_status_t status = SX126X_HW_API_SUCCESS;
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (latency_ms == NULL) {
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Update latency value.
    switch (latency_type) {
    case SX126X_HW_API_LATENCY_RESET:
        (*latency_ms) = SX126X_HAL_RESET_DELAY_MS;
        break;
    case SX126X_HW_API_LATENCY_WAKEUP:
        (*latency_ms) = SX126X_HAL_WAKEUP_DELAY_MS;
        break;
    default:
        SIGFOX_EXIT_ERROR(SX126X_HW_API_ERROR);
        break;
    }
errors:
    SIGFOX_RETURN();
}
#endif

#else
#error "SIGFOX EP LIB flags error: No RC compatible with SX1261MB1BAS shield."
#endif /* (defined SIGFOX_EP_RC1_ZONE) || (defined SIGFOX_EP_RC6_ZONE) || (defined SIGFOX_EP_RC7_ZONE) */