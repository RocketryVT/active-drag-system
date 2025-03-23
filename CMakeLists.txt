cmake_minimum_required(VERSION 3.12)

set(PICO_BOARD rp2040_micro)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include)
# Pull in SDK (must be before project)
include(${CMAKE_CURRENT_LIST_DIR}/include/pico-sdk/pico_sdk_init.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/include/pico-logger/CMakeLists.txt)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/eigen)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)

project(active-drag-system C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_subdirectory(src)   
add_subdirectory(tools)


# Pull in FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/include/FreeRTOS-Kernel)

set(FREERTOS_KERNEL_RP2040_RELATIVE_PATH "portable/ThirdParty/GCC/RP2040")

if (NOT FREERTOS_KERNEL_PATH)
    # check if we are inside the FreeRTOS kernel tree (i.e. this file has been included directly)
    get_filename_component(_ACTUAL_PATH ${CMAKE_CURRENT_LIST_DIR} REALPATH)
    get_filename_component(_POSSIBLE_PATH ${CMAKE_CURRENT_LIST_DIR}/${FREERTOS_KERNEL_RP2040_BACK_PATH}/${FREERTOS_KERNEL_RP2040_RELATIVE_PATH} REALPATH)
    if (_ACTUAL_PATH STREQUAL _POSSIBLE_PATH)
        get_filename_component(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/${FREERTOS_KERNEL_RP2040_BACK_PATH} REALPATH)
    endif()
    if (_ACTUAL_PATH STREQUAL _POSSIBLE_PATH)
        get_filename_component(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/${FREERTOS_KERNEL_RP2040_BACK_PATH} REALPATH)
        message("Setting FREERTOS_KERNEL_PATH to ${FREERTOS_KERNEL_PATH} based on location of FreeRTOS-Kernel-import.cmake")
    elseif (PICO_SDK_PATH AND EXISTS "${PICO_SDK_PATH}/../FreeRTOS-Kernel")
        set(FREERTOS_KERNEL_PATH ${PICO_SDK_PATH}/../FreeRTOS-Kernel)
        message("Defaulting FREERTOS_KERNEL_PATH as sibling of PICO_SDK_PATH: ${FREERTOS_KERNEL_PATH}")
    endif()
endif ()

if (NOT FREERTOS_KERNEL_PATH)
    foreach(POSSIBLE_SUFFIX Source FreeRTOS-Kernel FreeRTOS/Source)
        # check if FreeRTOS-Kernel exists under directory that included us
        set(SEARCH_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
        set(SEARCH_ROOT ../../../..)
        get_filename_component(_POSSIBLE_PATH ${SEARCH_ROOT}/${POSSIBLE_SUFFIX} REALPATH)
        if (EXISTS ${_POSSIBLE_PATH}/${FREERTOS_KERNEL_RP2040_RELATIVE_PATH}/CMakeLists.txt)
            get_filename_component(FREERTOS_KERNEL_PATH ${_POSSIBLE_PATH} REALPATH)
            message("Setting FREERTOS_KERNEL_PATH to '${FREERTOS_KERNEL_PATH}' found relative to enclosing project")
            break()
        endif()
    endforeach()
endif()

if (NOT FREERTOS_KERNEL_PATH)
    message(FATAL_ERROR "FreeRTOS location was not specified. Please set FREERTOS_KERNEL_PATH.")
endif()

set(FREERTOS_KERNEL_PATH "${FREERTOS_KERNEL_PATH}" CACHE PATH "Path to the FreeRTOS Kernel")

get_filename_component(FREERTOS_KERNEL_PATH "${FREERTOS_KERNEL_PATH}" REALPATH BASE_DIR "${CMAKE_BINARY_DIR}")
if (NOT EXISTS ${FREERTOS_KERNEL_PATH})
    message(FATAL_ERROR "Directory '${FREERTOS_KERNEL_PATH}' not found")
endif()
if (NOT EXISTS ${FREERTOS_KERNEL_PATH}/${FREERTOS_KERNEL_RP2040_RELATIVE_PATH}/CMakeLists.txt)
    message(FATAL_ERROR "Directory '${FREERTOS_KERNEL_PATH}' does not contain an RP2040 port here: ${FREERTOS_KERNEL_RP2040_RELATIVE_PATH}")
endif()
set(FREERTOS_KERNEL_PATH ${FREERTOS_KERNEL_PATH} CACHE PATH "Path to the FreeRTOS_KERNEL" FORCE)

add_subdirectory(${FREERTOS_KERNEL_PATH}/${FREERTOS_KERNEL_RP2040_RELATIVE_PATH} FREERTOS_KERNEL)
# End FreeRTOS

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
