cmake_minimum_required(VERSION 3.14)
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR ARM)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Set the cross-compiler for ARM32
if(CMAKE_HOST_APPLE)
    message(STATUS "Running on an Apple system")
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    include(CTest)
    # Apple Settings
    # Musl has many targets, must select one
    set(CMAKE_C_COMPILER /opt/homebrew/bin/arm-linux-musleabihf-gcc)
    set(CMAKE_CXX_COMPILER /opt/homebrew/bin/arm-linux-musleabihf-g++)
elseif(UNIX)
    message(STATUS "Running on an Linux system")

    # Linux Settings
    # (arm-linux-musleabihf- is 32 bit, aarch64-linux... is the 64 bit version)
    set(CMAKE_C_COMPILER arm-linux-musleabihf-gcc)
    set(CMAKE_CXX_COMPILER arm-linux-musleabihf-g++)
endif()

# Set the architecture and flags
# set(CMAKE_C_FLAGS "-march=armv7-a+fp")
# set(CMAKE_CXX_FLAGS "-march=armv7-a+fp")

# project(Active-Drag-System CXX)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/out")

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
