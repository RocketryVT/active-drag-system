cmake_minimum_required(VERSION 3.10)

set(PICO_BOARD rp2040_micro)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include)
# Pull in SDK (must be before project)
include(${CMAKE_CURRENT_LIST_DIR}/include/pico-sdk/pico_sdk_init.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/include/pico-logger/CMakeLists.txt)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/eigen)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

project(active-drag-system C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

# BEGIN libfixmath, libfixmatrix, and libfixkalman library generation
    add_library(libfixmath INTERFACE)

    file(GLOB libfixmath-srcs ${CMAKE_CURRENT_LIST_DIR}/include/libfixmath/libfixmath/*.c)
    target_sources(libfixmath INTERFACE ${libfixmath-srcs})
    target_include_directories(libfixmath INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/include/libfixmath/libfixmath
    )
    target_compile_definitions(libfixmath INTERFACE
            FIXMATH_FAST_SIN
            # FIXMATH_NO_64BIT
            # FIXMATH_SIN_LUT
            # FIXMATH_NO_CACHE
            # FIXMATH_NO_HARD_DIVISION
            FIXMATH_NO_OVERFLOW
            # FIXMATH_NO_ROUNDING
            # FIXMATH_OPTIMIZE_8BIT
    )

    add_library(libfixmatrix INTERFACE)

    file(GLOB libfixmatrix-srcs ${CMAKE_CURRENT_LIST_DIR}/include/libfixmatrix/*.c)
    list(FILTER libfixmatrix-srcs EXCLUDE REGEX ".*unittests\\.c$")
    target_sources(libfixmatrix INTERFACE ${libfixmatrix-srcs})
    target_include_directories(libfixmatrix INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/include/libfixmatrix
    )
    target_compile_definitions(libfixmatrix INTERFACE FIXMATRIX_MAX_SIZE=3)
    target_link_libraries(libfixmatrix INTERFACE libfixmath)

    add_library(libfixkalman INTERFACE)

    target_sources(libfixkalman INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include/libfixkalman/fixkalman.c)
    target_include_directories(libfixkalman INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include/libfixkalman)
    target_compile_definitions(libfixkalman INTERFACE
        KALMAN_DISABLE_UC
        KALMAN_TIME_VARYING
    )
    target_link_libraries(libfixkalman INTERFACE libfixmath libfixmatrix)
# END libfixmath, libfixmatrix, and libfixkalman library generation

add_subdirectory(src)
add_subdirectory(tools)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -fpermissive
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()
