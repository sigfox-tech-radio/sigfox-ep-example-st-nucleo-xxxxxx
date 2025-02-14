/*!*****************************************************************
 * \file    mcu.c
 * \brief   Common MCU driver based on LL driver.
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

#include "mcu.h"

#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#include "stm32wl3x_ll_utils.h"

/*** MCU functions ***/

/*******************************************************************/
MCAL_status_t MCU_get_die_id(uint8_t *die_id) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint8_t idx = 0;
    // Check parameters.
    if (die_id == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    for (idx = 0; idx < MCU_DIE_ID_SIZE_BYTES; idx++) {
        die_id[idx] = (uint8_t) (LL_GetDIE_ID() >> (idx << 3));
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t MCU_get_chip_id(uint8_t *chip_id) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    uint8_t idx = 0;
    // Check parameters.
    if (chip_id == NULL) {
        status = MCAL_ERROR;
        goto errors;
    }
    for (idx = 0; idx < MCU_DIE_ID_SIZE_BYTES; idx++) {
        if (idx < 4) {
            chip_id[idx] = (uint8_t) (LL_GetUID_Word0() >> ((idx % 4) << 3));
        }
        else {
            chip_id[idx] = (uint8_t) (LL_GetUID_Word1() >> ((idx % 4) << 3));
        }
    }
errors:
    return status;
}
