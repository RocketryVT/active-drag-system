if (COMPILE_TOOLS)
    add_executable(read_flash
                   read_flash.cpp
                   ${PROJECT_SOURCE_DIR}/src/ms5607.cpp
                   ${PROJECT_SOURCE_DIR}/src/adxl375.cpp
                   ${PROJECT_SOURCE_DIR}/src/iim42653.cpp
                   ${PROJECT_SOURCE_DIR}/src/mmc5983ma.cpp
                   ${PROJECT_SOURCE_DIR}/src/log_format.cpp
                   ${PROJECT_SOURCE_DIR}/src/serial.cpp
    )

    add_executable(servo_test
                   servo_test.cpp
                   ${PROJECT_SOURCE_DIR}/src/pwm.cpp
    )

    add_executable(alt_test
                   alt_test.cpp
                   ${PROJECT_SOURCE_DIR}/src/ms5607.cpp
                   ${PROJECT_SOURCE_DIR}/src/heartbeat.cpp
    )

    # pull in common dependencies
    target_link_libraries(read_flash pico_stdlib pico_logger pico_flash pico_rand pico_multicore pico_sync hardware_i2c hardware_adc hardware_timer FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)
    target_include_directories(read_flash PUBLIC ${PROJECT_SOURCE_DIR}/include)
    target_compile_definitions(read_flash PRIVATE
            USE_FREERTOS=1
            DEBUG=1
            PICO_STDIO_STACK_BUFFER_SIZE=64 # use a small printf on stack buffer
    )
    pico_set_binary_type(read_flash copy_to_ram)

    target_link_libraries(servo_test pico_stdlib hardware_pwm hardware_i2c)
    target_include_directories(servo_test PUBLIC ${PROJECT_SOURCE_DIR}/include)

    target_link_libraries(alt_test pico_stdlib hardware_i2c hardware_gpio)
    target_include_directories(alt_test PUBLIC ${PROJECT_SOURCE_DIR}/include)
    
    pico_enable_stdio_usb(read_flash 1)
    pico_enable_stdio_uart(read_flash 0)

    pico_enable_stdio_usb(servo_test 1)
    pico_enable_stdio_uart(servo_test 0)

    pico_enable_stdio_usb(alt_test 1)
    pico_enable_stdio_uart(alt_test 0)
    
    # create map/bin/hex file etc.
    pico_add_extra_outputs(read_flash)
    pico_add_extra_outputs(servo_test)
    pico_add_extra_outputs(alt_test)
endif()

