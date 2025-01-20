/*!*****************************************************************
 * \file    at_hw_api.c
 * \brief   AT low level interface.
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

#include "at_hw_api.h"

#include "at.h"
#include "stdint.h"
#include "mcal.h"
#include "usart.h"



/*** AT HW API functions ***/

/*******************************************************************/
#define _check_mcal_status(void) { if (mcal_status != MCAL_SUCCESS) { status = AT_ERROR_AT_HW_API; goto errors; } }

/*******************************************************************/
AT_status_t AT_HW_API_init(AT_HW_API_config_t *hw_api_config) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    MCAL_status_t mcal_status = MCAL_SUCCESS;

    // Init USART interface.
    mcal_status = USART_init(115200, 0, hw_api_config->rx_irq_callback);
    _check_mcal_status();
errors:
    return status;
}

/*******************************************************************/
AT_status_t AT_HW_API_de_init(void) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    // Release USART interface.
    USART_de_init();
    return status;
}

/*******************************************************************/
AT_status_t AT_HW_API_write(uint8_t *data, uint32_t data_size_bytes) {
    // Local variables.
    AT_status_t status = AT_SUCCESS;
    MCAL_status_t mcal_status = MCAL_SUCCESS;

    mcal_status = USART_write( data, data_size_bytes);
    _check_mcal_status();
errors:
    return status;
}
