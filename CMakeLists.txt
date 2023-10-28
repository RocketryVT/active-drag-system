cmake_minimum_required(VERSION 3.14)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(Active-Drag-System CXX)

# Set the cross-compiler for ARM32
if(CMAKE_HOST_APPLE)
    message(STATUS "Running on an Apple system")

    # Apple Settings 
    set(CMAKE_C_COMPILER /opt/homebrew/bin/arm-linux-musleabihf-gcc)
    set(CMAKE_CXX_COMPILER /opt/homebrew/bin/arm-linux-musleabihf-g++)
elseif(UNIX)
    message(STATUS "Running on an Linux system")

    # Linux Settings
    set(CMAKE_C_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
    set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)
endif()

# Set the architecture and flags
set(CMAKE_C_FLAGS "-march=armv7-a+fp")
set(CMAKE_CXX_FLAGS "-march=armv7-a+fp")

# sysroot location; mounted using sshfs
# set(CMAKE_SYSROOT "/vagrant/bbb-sysroot")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")

# look in beaglebone for libraries and not host system
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

add_subdirectory(src)

if(BUILD_TESTING)
    include(CTest)

    add_subdirectory(test)

    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
    )

    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

