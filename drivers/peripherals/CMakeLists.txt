################################################################################
#                      _____ _____ _____ ______ ______   __                    #
#                     / ____|_   _/ ____|  ____/ __ \ \ / /                    #
#                    | (___   | || |  __| |__ | |  | \ V /                     #
#                     \___ \  | || | |_ |  __|| |  | |> <                      #
#                     ____) |_| || |__| | |   | |__| / . \                     #
#                    |_____/|_____\_____|_|    \____/_/ \_\                    #
#                                  ___   _____                                 #
#                                 / _ \ / ____|                                #
#                                | | | | |  __                                 #
#                                | | | | | |_ |                                #
#                                | |_| | |__| |                                #
#                                 \___/ \_____|                                #
#     _______ ______ _____ _    _ _   _  ____  _      ____   _______     __    #
#    |__   __|  ____/ ____| |  | | \ | |/ __ \| |    / __ \ / ____\ \   / /    #
#       | |  | |__ | |    | |__| |  \| | |  | | |   | |  | | |  __ \ \_/ /     #
#       | |  |  __|| |    |  __  | . ` | |  | | |   | |  | | | |_ | \   /      #
#       | |  | |___| |____| |  | | |\  | |__| | |___| |__| | |__| |  | |       #
#       |_|  |______\_____|_|  |_|_| \_|\____/|______\____/ \_____|  |_|       #
#             ______     __  _    _ _   _          ____ _____ ______           #
#            |  _ \ \   / / | |  | | \ | |   /\   |  _ \_   _|___  /           #
#            | |_) \ \_/ /  | |  | |  \| |  /  \  | |_) || |    / /            #
#            |  _ < \   /   | |  | | . ` | / /\ \ |  _ < | |   / /             #
#            | |_) | | |    | |__| | |\  |/ ____ \| |_) || |_ / /__            #
#            |____/  |_|     \____/|_| \_/_/    \_\____/_____/_____|           #
#                                                                              #
################################################################################
#
# Copyright (c) 2024, UnaBiz SAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1 Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  2 Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################
 
cmake_minimum_required(VERSION 3.21)
project(peripherials)

add_library(${PROJECT_NAME} OBJECT)

if(BOARD_NAME STREQUAL nucleo-l053r8)
    target_sources(${PROJECT_NAME} 
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_adc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_pwr.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_exti.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_lptim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_rcc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_spi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_ll_usart.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Src/stm32l0xx_hal_flash.c
    )    
    target_include_directories(${PROJECT_NAME} 
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32l0xx-hal-driver/Inc/Legacy
    )
    target_compile_definitions(${PROJECT_NAME} 
        PUBLIC 
        USE_FULL_LL_DRIVER
    )
elseif(BOARD_NAME STREQUAL nucleo-wl33cc1)
    target_sources(${PROJECT_NAME} 
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_adc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_pwr.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_rcc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_system.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_usart.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_ll_utils.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_hal.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_hal_flash.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_hal_flash_ex.c
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Src/stm32wl3x_hal_mrsubg.c
    )    
    target_include_directories(${PROJECT_NAME} 
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/stm32wl3x-hal-driver/Inc/Legacy
    )
    target_compile_definitions(${PROJECT_NAME} 
        PUBLIC 
        USE_FULL_LL_DRIVER
    )                 
endif()

target_sources(${PROJECT_NAME} 
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/adc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/exti.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/lptim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/nvm.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/pwr.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/rcc.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/spi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/usart.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mcal/src/${BOARD_NAME}/mcu.c
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:cmsis,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_INCLUDE_DIRECTORIES>>
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/mcal/inc
    ${CMAKE_CURRENT_SOURCE_DIR}            
)
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:cmsis,INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_COMPILE_DEFINITIONS>>
)
