cmake_minimum_required(VERSION 3.14)
project(Active-Drag-System CXX)

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER /usr/bin/arm-linux-gnueabihf-g++)

# sysroot location; mounted using sshfs
set(CMAKE_SYSROOT "/vagrant/bbb-sysroot")

# look in beaglebone for libraries and not host system
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

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


add_subdirectory(src)

