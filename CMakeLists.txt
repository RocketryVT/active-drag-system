cmake_minimum_required(VERSION 3.10)

set(PICO_BOARD rp2040_micro)
set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include)

# Shared dependency locations (projects/libs/Third_Party by default)
get_filename_component(AVIONICS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
include(${AVIONICS_ROOT}/cmake/deps.cmake)

set(PROJECT_THIRD_PARTY_DIR ${CMAKE_CURRENT_LIST_DIR}/include)

# Pull in SDK (must be before project)
set(PICO_SDK_ROOT ${PICO_SDK_PATH})
if (NOT EXISTS "${PICO_SDK_ROOT}/pico_sdk_init.cmake")
    message(FATAL_ERROR "Pico SDK not found at ${PICO_SDK_ROOT}. Initialize the submodule or set PICO_SDK_PATH.")
endif()
include(${PICO_SDK_ROOT}/pico_sdk_init.cmake)

set(FREERTOS_KERNEL_ROOT ${FREERTOS_KERNEL_PATH})
if (NOT EXISTS "${FREERTOS_KERNEL_ROOT}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake")
    message(FATAL_ERROR "FreeRTOS Kernel not found at ${FREERTOS_KERNEL_ROOT}. Initialize the submodule or set FREERTOS_KERNEL_PATH.")
endif()
set(FREERTOS_THIRD_PARTY_DIR ${FREERTOS_KERNEL_ROOT}/portable/ThirdParty)
include(${FREERTOS_THIRD_PARTY_DIR}/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

set(PICO_LOGGER_ROOT ${PICO_LOGGER_PATH})
if (NOT EXISTS "${PICO_LOGGER_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "pico-logger not found at ${PICO_LOGGER_ROOT}. Initialize the submodule or set PICO_LOGGER_PATH.")
endif()

set(EIGEN_ROOT ${EIGEN_PATH})
if (NOT EXISTS "${EIGEN_ROOT}/Eigen")
    message(FATAL_ERROR "Eigen not found at ${EIGEN_ROOT}. Initialize the submodule or set EIGEN_PATH.")
endif()
include_directories(${EIGEN_ROOT})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_OSX_ARCHITECTURES "" CACHE STRING "Disable host -arch flag when cross-compiling for bare metal" FORCE)
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

add_subdirectory(${PICO_LOGGER_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/pico-logger)

# BEGIN libfixmath, libfixmatrix, and libfixkalman library generation
    add_library(libfixmath INTERFACE)

    set(LIBFIXMATH_ROOT ${LIBFIXMATH_PATH})
    if (NOT EXISTS "${LIBFIXMATH_ROOT}/libfixmath")
        message(FATAL_ERROR "libfixmath not found at ${LIBFIXMATH_ROOT}. Initialize the submodule or set LIBFIXMATH_PATH.")
    endif()

    file(GLOB libfixmath-srcs ${LIBFIXMATH_ROOT}/libfixmath/*.c)
    target_sources(libfixmath INTERFACE ${libfixmath-srcs})
    target_include_directories(libfixmath INTERFACE
            ${LIBFIXMATH_ROOT}/libfixmath
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

    set(LIBFIXMATRIX_ROOT ${LIBFIXMATRIX_PATH})
    if (NOT EXISTS "${LIBFIXMATRIX_ROOT}")
        message(FATAL_ERROR "libfixmatrix not found at ${LIBFIXMATRIX_ROOT}. Initialize the submodule or set LIBFIXMATRIX_PATH.")
    endif()

    file(GLOB libfixmatrix-srcs ${LIBFIXMATRIX_ROOT}/*.c)
    list(FILTER libfixmatrix-srcs EXCLUDE REGEX ".*unittests\\.c$")
    target_sources(libfixmatrix INTERFACE ${libfixmatrix-srcs})
    target_include_directories(libfixmatrix INTERFACE
            ${LIBFIXMATRIX_ROOT}
    )
    target_compile_definitions(libfixmatrix INTERFACE FIXMATRIX_MAX_SIZE=3)
    target_link_libraries(libfixmatrix INTERFACE libfixmath)

    add_library(libfixkalman INTERFACE)

    set(LIBFIXKALMAN_ROOT ${LIBFIXKALMAN_PATH})
    if (NOT EXISTS "${LIBFIXKALMAN_ROOT}")
        message(FATAL_ERROR "libfixkalman not found at ${LIBFIXKALMAN_ROOT}. Initialize the submodule or set LIBFIXKALMAN_PATH.")
    endif()

    target_sources(libfixkalman INTERFACE ${LIBFIXKALMAN_ROOT}/fixkalman.c)
    target_include_directories(libfixkalman INTERFACE ${LIBFIXKALMAN_ROOT})
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
