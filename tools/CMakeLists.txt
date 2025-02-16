if (COMPILE_TOOLS)
    add_executable(read_flash
                   read_flash.c
                   ../src/spi_flash.c
    )

    add_executable(servo_test
                   servo_test.cpp
                   ../src/pwm.cpp
    )

    add_executable(alt_test
                   alt_test.cpp
    )

    add_executable(imu_calib
                   imu_calib.cpp
    )

    add_executable(board_test
                   board_test.cpp
    )

    add_executable(motor_driver
                   motor_driver.cpp
    )

    add_executable(pwm_measure
                   pwm_measure.cpp
                   measure_pwm.pio
    )
    pico_generate_pio_header(pwm_measure ${CMAKE_CURRENT_LIST_DIR}/measure_pwm.pio)
    # pull in common dependencies
    target_link_libraries(read_flash pico_stdlib hardware_spi)
    target_include_directories(read_flash PUBLIC ../include)

    target_link_libraries(servo_test pico_stdlib hardware_pwm hardware_i2c)
    target_include_directories(servo_test PUBLIC ../include)

    target_link_libraries(alt_test pico_stdlib hardware_i2c hardware_gpio)

    target_link_libraries(board_test pico_stdlib hardware_i2c hardware_clocks hardware_gpio)

    target_link_libraries(imu_calib pico_stdlib hardware_i2c hardware_gpio)
    target_include_directories(imu_calib PUBLIC ../include)

    target_link_libraries(motor_driver pico_stdlib pico_multicore pico_binary_info hardware_i2c pico_divider hardware_spi hardware_clocks hardware_gpio hardware_pwm)

    target_link_libraries(pwm_measure pico_stdlib hardware_gpio hardware_pio hardware_clocks hardware_pwm) 

    pico_enable_stdio_usb(read_flash 1)
    pico_enable_stdio_uart(read_flash 0)

    pico_enable_stdio_usb(servo_test 1)
    pico_enable_stdio_uart(servo_test 0)

    pico_enable_stdio_usb(alt_test 1)
    pico_enable_stdio_uart(alt_test 0)

    pico_enable_stdio_usb(board_test 1)
    pico_enable_stdio_uart(board_test 0)

    pico_enable_stdio_usb(imu_calib 1)
    pico_enable_stdio_uart(imu_calib 0)

    pico_enable_stdio_usb(motor_driver 1)
    pico_enable_stdio_uart(motor_driver 0)

    pico_enable_stdio_usb(pwm_measure 1)
    pico_enable_stdio_uart(pwm_measure 0)

    # create map/bin/hex file etc.
    pico_add_extra_outputs(read_flash)
    pico_add_extra_outputs(servo_test)
    pico_add_extra_outputs(alt_test)
    pico_add_extra_outputs(board_test)
    pico_add_extra_outputs(imu_calib)
    pico_add_extra_outputs(motor_driver)
    pico_add_extra_outputs(pwm_measure)
endif()

