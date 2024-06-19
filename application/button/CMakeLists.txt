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
project(app-button)

list(APPEND ${PROJECT_NAME}_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main_button.c
)

add_executable(${PROJECT_NAME}
#add_library(${PROJECT_NAME} OBJECT
    ${${PROJECT_NAME}_SOURCES}
    $<TARGET_OBJECTS:sigfox_ep_lib_obj>
    $<TARGET_OBJECTS:${SIGFOX_RF_API}>
    $<TARGET_OBJECTS:mcu_api>
    $<TARGET_OBJECTS:peripherials>
    $<TARGET_OBJECTS:cmsis>
    $<TARGET_OBJECTS:shield>
    $<TARGET_OBJECTS:utils> 
    $<TARGET_OBJECTS:components>
    $<TARGET_OBJECTS:${RADIO_DRIVER}>  
)

target_include_directories( ${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:${SIGFOX_RF_API},INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:mcu_api,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:peripherials,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:cmsis,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:shield,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:utils,INTERFACE_INCLUDE_DIRECTORIES>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:components,INTERFACE_INCLUDE_DIRECTORIES>>
)

target_compile_definitions(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:sigfox_ep_lib_obj,INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:${SIGFOX_RF_API},INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:mcu_api,INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:peripherials,INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:cmsis,INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:shield,INTERFACE_COMPILE_DEFINITIONS>>
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:components,INTERFACE_COMPILE_DEFINITIONS>>
)

target_link_options(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:$<TARGET_PROPERTY:cmsis,INTERFACE_LINK_OPTIONS>>
    "LINKER:-Map,${PROJECT_NAME}.map"
)

set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

target_link_libraries(${PROJECT_NAME} gcc c_nano )

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD 
    COMMAND ${CMAKE_SIZE_UTIL} ${PROJECT_NAME}.elf
)

add_custom_target(${PROJECT_NAME}.hex ALL 
    DEPENDS ${PROJECT_NAME}
    BYPRODUCTS ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -Oihex  ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
)

add_custom_target(${PROJECT_NAME}.bin ALL
    DEPENDS ${PROJECT_NAME}
    BYPRODUCTS ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -Obinary  ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)