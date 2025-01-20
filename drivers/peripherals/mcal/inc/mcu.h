/*!*****************************************************************
 * \file    mcu.h
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

#ifndef __MCU_H__
#define __MCU_H__

#include "mcal.h"
#include "stdint.h"

/*** MCU macros ***/

#define MCU_DIE_ID_SIZE_BYTES   4
#define MCU_CHIP_ID_SIZE_BYTES  8

/*** MCU functions ***/

/*!******************************************************************
 * \fn MCAL_status_t MCU_get_die_id(uint8_t *die_id);
 * \brief Read MCU DIE id.
 * \param[in]   none
 * \param[out]  die_id: Pointer to the DIE ID bytes.
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t MCU_get_die_id(uint8_t *die_id);

/*!******************************************************************
 * \fn MCAL_status_t MCU_get_chip_id(uint8_t *chip_id);
 * \brief Read MCU chip id.
 * \param[in]   none
 * \param[out]  die_id: Pointer to the chip ID bytes.
 * \retval      Function execution status.
 *******************************************************************/
MCAL_status_t MCU_get_chip_id(uint8_t *chip_id);

#endif /* __MCU_H__ */
