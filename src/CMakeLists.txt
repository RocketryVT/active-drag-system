add_executable(ads
        active_drag_system.cpp
        spi_flash.c
        bno055.cpp
        SimpleKalmanFilter.cpp
        # kalmanfilter.cpp
        pwm.cpp
        AltEst/algebra.cpp
        AltEst/altitude.cpp
        AltEst/filters.cpp
        )

add_executable(read_flash
        read_flash.c
        spi_flash.c
        )

add_executable(servo_test
        servo_test.cpp
        pwm.cpp
        )

add_executable(alt_test
        altimeter.cpp
        )

# pull in common dependencies
target_link_libraries(ads pico_stdlib pico_multicore pico_sync hardware_i2c hardware_spi hardware_pwm ${Eigen_LIBRARIES})
target_include_directories(ads PUBLIC ../include)

target_link_libraries(read_flash pico_stdlib hardware_spi)
target_include_directories(read_flash PUBLIC ../include)

target_link_libraries(servo_test pico_stdlib hardware_pwm hardware_i2c)
target_include_directories(servo_test PUBLIC ../include)

target_link_libraries(alt_test pico_stdlib hardware_i2c hardware_gpio)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

pico_enable_stdio_usb(read_flash 1)
pico_enable_stdio_uart(read_flash 0)

pico_enable_stdio_usb(servo_test 1)
pico_enable_stdio_uart(servo_test 0)

pico_enable_stdio_usb(alt_test 1)
pico_enable_stdio_uart(alt_test 0)
# create map/bin/hex file etc.
pico_add_extra_outputs(ads)
pico_add_extra_outputs(read_flash)
pico_add_extra_outputs(servo_test)
pico_add_extra_outputs(alt_test)

