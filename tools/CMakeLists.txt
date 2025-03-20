if (COMPILE_TOOLS)
    add_executable(read_flash
                   read_flash.c
                   ${PROJECT_SOURCE_DIR}/src/spi_flash.c
    )

    add_executable(servo_test
                   servo_test.cpp
                   ${PROJECT_SOURCE_DIR}/src/pwm.cpp
    )

    add_executable(alt_test
                   alt_test.cpp
                   ${PROJECT_SOURCE_DIR}/src/altimeter.cpp
                   ${PROJECT_SOURCE_DIR}/src/heartbeat.cpp
    )

    add_executable(imu_calib
                   imu_calib.cpp
    )

	add_executable(clock_gen_test
                   clock_gen_test.cpp
	)

	add_executable(measurement_test
                   measurement_test.cpp
                   ${PROJECT_SOURCE_DIR}/src/mid_imu.cpp
                   ${PROJECT_SOURCE_DIR}/src/high_accel.cpp
                   ${PROJECT_SOURCE_DIR}/src/magnetometer.cpp
                   ${PROJECT_SOURCE_DIR}/src/sensor_i2c.cpp
	)

    # pull in common dependencies
    target_link_libraries(read_flash pico_stdlib hardware_spi)
    target_include_directories(read_flash PUBLIC ${PROJECT_SOURCE_DIR}/include)

    target_link_libraries(servo_test pico_stdlib hardware_pwm hardware_i2c)
    target_include_directories(servo_test PUBLIC ${PROJECT_SOURCE_DIR}/include)

    target_link_libraries(alt_test pico_stdlib hardware_i2c hardware_gpio)
    target_include_directories(alt_test PUBLIC ${PROJECT_SOURCE_DIR}/include)

    target_link_libraries(imu_calib pico_stdlib hardware_i2c hardware_gpio)
    target_include_directories(imu_calib PUBLIC ${PROJECT_SOURCE_DIR}/include)
	
    target_link_libraries(clock_gen_test pico_stdlib hardware_clocks hardware_gpio)
	
    target_link_libraries(measurement_test pico_stdlib pico_time hardware_gpio hardware_i2c hardware_clocks ${Eigen_LIBRARIES})
    target_include_directories(measurement_test PUBLIC ${PROJECT_SOURCE_DIR}/include)

    pico_enable_stdio_usb(read_flash 1)
    pico_enable_stdio_uart(read_flash 0)

    pico_enable_stdio_usb(servo_test 1)
    pico_enable_stdio_uart(servo_test 0)

    pico_enable_stdio_usb(alt_test 1)
    pico_enable_stdio_uart(alt_test 0)

    pico_enable_stdio_usb(imu_calib 1)
    pico_enable_stdio_uart(imu_calib 0)
	
    pico_enable_stdio_usb(measurement_test 1)
    pico_enable_stdio_uart(measurement_test 0)

    # create map/bin/hex file etc.
    pico_add_extra_outputs(read_flash)
    pico_add_extra_outputs(servo_test)
    pico_add_extra_outputs(alt_test)
    pico_add_extra_outputs(imu_calib)
    pico_add_extra_outputs(clock_gen_test)
    pico_add_extra_outputs(measurement_test)
endif()

