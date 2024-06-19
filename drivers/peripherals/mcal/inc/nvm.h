/*!*****************************************************************
 * \file    nvm.h
 * \brief   Common NVM driver based on LL driver.
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

#ifndef __NVM_H__
#define __NVM_H__

#include "mcal.h"
#include "sigfox_types.h"
#include "stdint.h"

/*** NVM macros ***/

#define NVM_SIZE_BYTES		2048

/*** NVM structures ***/

/*!******************************************************************
 * \enum NVM_address_t
 * \brief NVM address mapping.
 *******************************************************************/
typedef enum {
	NVM_ADDRESS_SIGFOX_EP_LIB_DATA = 0,
	NVM_ADDRESS_LAST = (NVM_SIZE_BYTES - 1)
} NVM_address_t;

/*** NVM functions ***/

/*!******************************************************************
 * \fn MCAL_status_t NVM_read_byte(NVM_address_t address, uint8_t* data)
 * \brief Read byte in NVM.
 * \param[in]  	address: Address to read.
 * \param[out] 	data: Pointer to byte that will contain the read value.
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t NVM_read_byte(NVM_address_t address, uint8_t* data);

/*!******************************************************************
 * \fn MCAL_status_t NVM_write_byte(NVM_address_t address, uint8_t data)
 * \brief Write byte in NVM.
 * \param[in]  	address: Address to write.
 * \param[out] 	data: Byte to write.
 * \retval		Function execution status.
 *******************************************************************/
MCAL_status_t NVM_write_byte(NVM_address_t address, uint8_t data);

#endif /* __NVM_H__ */
