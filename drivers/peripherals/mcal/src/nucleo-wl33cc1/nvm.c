/*!*****************************************************************
 * \file    nvm.c
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

#include "nvm.h"

#include "mcal.h"
#include "stddef.h"
#include "stdint.h"
#include "stm32wl3x_hal_def.h"
#include "stm32wl3x_hal_flash.h"
#include "stm32wl3x_ll_bus.h"
#include "system_stm32wl3x.h"

/*** NVM linker generated symbols ***/

extern uint32_t _MEMORY_EEPROM_BEGIN_;

/*** NVM local macros ***/

#define NVM_SIZE_BYTES                  NVM_ADDRESS_LAST

#define NVM_WORD_SIZE_BYTES             4
#define NVM_WORD_BLANK                  0xFFFFFFFF
#define NVM_BYTE_BLANK                  0xFF

#define NVM_PAGE_SIZE_WORDS             (FLASH_PAGE_SIZE / NVM_WORD_SIZE_BYTES)

#define NVM_EEPROM_PAGE_ADDRESS         ((uint32_t) &_MEMORY_EEPROM_BEGIN_)
#define NVM_EEPROM_PAGE_INDEX           ((NVM_EEPROM_PAGE_ADDRESS - _MEMORY_FLASH_BEGIN_) / (FLASH_PAGE_SIZE))

/*** NVM local structures ***/

/*******************************************************************/
typedef enum {
    NVM_MEMORY_STATUS_EMPTY = 0,
    NVM_MEMORY_STATUS_NOT_EMPTY,
    NVM_MEMORY_STATUS_FULL
} NVM_memory_status_t;

/*******************************************************************/
typedef union {
    struct {
        uint32_t virtual_address : 24;
        uint32_t value : 8;
    } __attribute__((scalar_storage_order("big-endian")))__attribute__((packed)) field;
    uint32_t word;
} NVM_record_t;

/*******************************************************************/
typedef struct {
    uint8_t flash_erase_count;
} NVM_context_t;

/*** NVM local global variables ***/

static NVM_context_t nvm_ctx = { .flash_erase_count = 0 };

/*** NVM local functions ***/

/*******************************************************************/
#define _NVM_read_word(p_word, p_absolute_address) { p_word = *((uint32_t*) p_absolute_address); }

/*******************************************************************/
static NVM_memory_status_t _NVM_update_memory_status(uint32_t* free_absolute_address) {
    // Local variables.
    NVM_memory_status_t memory_status = NVM_MEMORY_STATUS_FULL;
    uint32_t absolute_address = 0;
    uint32_t word = 0;
    uint32_t idx = 0;
    // Search first blank word.
    for (idx = 0; idx < NVM_PAGE_SIZE_WORDS; idx++) {
        // Compute absolute address.
        absolute_address = (NVM_EEPROM_PAGE_ADDRESS + (idx * NVM_WORD_SIZE_BYTES));
        // Read word.
        _NVM_read_word(word, absolute_address);
        // Check if address is free.
        if (word == NVM_WORD_BLANK) {
            // Save address.
            (*free_absolute_address) = absolute_address;
            // Update status.
            memory_status = (idx == 0) ? NVM_MEMORY_STATUS_EMPTY : NVM_MEMORY_STATUS_NOT_EMPTY;
            break;
        }
    }
    return memory_status;
}

/*******************************************************************/
static uint8_t _NVM_search_last_record(NVM_address_t address, NVM_record_t* nvm_record) {
    // Local variables.
    NVM_record_t local_nvm_record;
    uint8_t record_found = 0;
    uint32_t idx = 0;
    // Read all page to retrieve the last value of the given address.
    for (idx = 0; idx < NVM_PAGE_SIZE_WORDS; idx++) {
        // Read word.
        _NVM_read_word(local_nvm_record.word, (NVM_EEPROM_PAGE_ADDRESS + (idx * NVM_WORD_SIZE_BYTES)));
        // Check address match.
        if (local_nvm_record.field.virtual_address == ((uint32_t) address)) {
            // Update flag.
            record_found = 1;
            // Update output value.
            nvm_record->word = local_nvm_record.word;
            // Note: do not break here since we want the last value stored in flash.
        }
    }
    return record_found;
}

/*** NVM functions ***/

/*******************************************************************/
MCAL_status_t NVM_read_byte(NVM_address_t address, uint8_t *data) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    NVM_record_t nvm_record;
    uint8_t record_found = 0;
    // Check parameters.
    if ((address >= NVM_SIZE_BYTES) || (data == NULL)) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Reset output data.
    (*data) = NVM_BYTE_BLANK;
    // Search data.
    record_found = _NVM_search_last_record(address, &nvm_record);
    // Check flag.
    if (record_found != 0) {
        (*data) = ((uint8_t) nvm_record.field.value);
    }
errors:
    return status;
}

/*******************************************************************/
MCAL_status_t NVM_write_byte(NVM_address_t address, uint8_t data) {
    // Local variables.
    MCAL_status_t status = MCAL_SUCCESS;
    HAL_StatusTypeDef hal_status = HAL_OK;
    NVM_memory_status_t memory_status = NVM_MEMORY_STATUS_FULL;
    uint32_t free_absolute_address = 0;
    NVM_record_t new_nvm_record;
    NVM_record_t backup_nvm_record[NVM_SIZE_BYTES];
    uint8_t data_byte = 0;
    uint8_t idx = 0;
    // Check parameters.
    if (address >= NVM_SIZE_BYTES) {
        status = MCAL_ERROR;
        goto errors;
    }
    // Build word to record.
    new_nvm_record.field.virtual_address = address;
    new_nvm_record.field.value = data;
    // Search free address.
    memory_status = _NVM_update_memory_status(&free_absolute_address);
    // Check status.
    if (memory_status == NVM_MEMORY_STATUS_FULL) {
        // Prevent from recursive call error (memory should not be full two consecutive times).
        if (nvm_ctx.flash_erase_count > 0) {
            status = MCAL_ERROR;
            goto errors;
        }
        // Backup all virtual addresses value.
        for (idx = 0; idx < NVM_ADDRESS_LAST; idx++) {
            // Read byte.
            status = NVM_read_byte(idx, &data_byte);
            if (status != MCAL_SUCCESS) goto errors;
            // Store in RAM backup.
            backup_nvm_record[idx].field.virtual_address = idx;
            backup_nvm_record[idx].field.value = (idx == address) ? data : data_byte;
        }
        // Erase page.
        // Note: an additional flash page could be used as data mirror in case a reset occurs during the RAM backup process. This is not implemented here.
        FLASH_PageErase(NVM_EEPROM_PAGE_INDEX);
        nvm_ctx.flash_erase_count++;
        // Restore backup and new value.
        for (idx = 0; idx < NVM_ADDRESS_LAST; idx++) {
            // Write byte.
            status = NVM_write_byte(backup_nvm_record[idx].field.virtual_address, backup_nvm_record[idx].field.value);
            if (status != MCAL_SUCCESS) goto errors;
        }
    }
    else {
        // Write word at free location.
        hal_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, free_absolute_address, new_nvm_record.word);
        if (hal_status != HAL_OK) {
            status = MCAL_ERROR;
            goto errors;
        }
    }
errors:
    nvm_ctx.flash_erase_count = 0;
    return status;
}
