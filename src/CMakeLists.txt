add_executable(ads
        active_drag_system.cpp
        bno055.cpp
        kalmanfilter.cpp
        pwm.cpp
        # spi_flash.c
        )

# pull in common dependencies
target_link_libraries(ads pico_stdlib hardware_i2c hardware_pwm hardware_spi pico_multicore ${Eigen_LIBRARIES})
target_include_directories(ads PUBLIC ../include)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ads)

