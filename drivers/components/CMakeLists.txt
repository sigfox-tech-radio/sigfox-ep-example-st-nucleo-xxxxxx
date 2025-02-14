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
project(components)

add_library(${RADIO_DRIVER} OBJECT EXCLUDE_FROM_ALL)

if(RADIO_DRIVER STREQUAL lr11xx_driver)
    target_sources(${RADIO_DRIVER}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/SWDR001/src/lr11xx_radio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/SWDR001/src/lr11xx_driver_version.c
        ${CMAKE_CURRENT_SOURCE_DIR}/SWDR001/src/lr11xx_radio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/SWDR001/src/lr11xx_regmem.c
        ${CMAKE_CURRENT_SOURCE_DIR}/SWDR001/src/lr11xx_system.c  
    )
    target_include_directories(${RADIO_DRIVER}
      PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/SWDR001/src/
    )
    target_compile_definitions(${RADIO_DRIVER} 
        PUBLIC 
        LR11XX_DISABLE_WARNINGS
    )
elseif(RADIO_DRIVER STREQUAL sx126x_driver)
    target_sources(${RADIO_DRIVER}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_driver/src/sx126x.c
        ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_driver/src/sx126x_driver_version.c
    )
    target_include_directories(${RADIO_DRIVER}
      PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/sx126x_driver/src/
    )
    target_compile_definitions(${RADIO_DRIVER} 
        PUBLIC 
    )
elseif(RADIO_DRIVER STREQUAL s2lp_driver)
    target_sources(${RADIO_DRIVER}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/s2lp.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Commands.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Fifo.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_General.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Gpio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_PacketHandler.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_PktBasic.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_PktWMbus.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Qi.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Radio.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Timer.c
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/src/S2LP_Types.c
    )
    target_include_directories(${RADIO_DRIVER}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/x-cube-subg2/Drivers/BSP/Components/S2LP/S2LP_Library/inc/
        $<TARGET_PROPERTY:s2lp_rf_api_obj,INTERFACE_INCLUDE_DIRECTORIES>
    )
    target_compile_definitions(${RADIO_DRIVER}
        PRIVATE
        $<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_COMPILE_DEFINITIONS>
    )
elseif(RADIO_DRIVER STREQUAL stm32wl3x_driver)
    target_sources(${RADIO_DRIVER}
        PUBLIC
        $<TARGET_OBJECTS:peripherials>
    )
    target_include_directories(${RADIO_DRIVER}
        PUBLIC
        $<TARGET_PROPERTY:peripherials,INTERFACE_INCLUDE_DIRECTORIES>
    )
    target_compile_definitions(${RADIO_DRIVER}
        PRIVATE
        $<TARGET_PROPERTY:peripherials,INTERFACE_COMPILE_DEFINITIONS>
    )
endif()

add_library(${PROJECT_NAME} OBJECT EXCLUDE_FROM_ALL)

target_sources(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/button/src/button.c
    $<TARGET_OBJECTS:${RADIO_DRIVER}>
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/button/inc/
    $<TARGET_PROPERTY:${RADIO_DRIVER},INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:peripherials,INTERFACE_INCLUDE_DIRECTORIES>
)

target_compile_definitions(${PROJECT_NAME}
    PUBLIC
    $<TARGET_PROPERTY:${RADIO_DRIVER},INTERFACE_COMPILE_DEFINITIONS>
)
