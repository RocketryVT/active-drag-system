add_executable(ads
        active_drag_system.cpp
        spi_flash.c
        bno055.cpp
        kalmanfilter.cpp
        pwm.cpp
        )

add_executable(read_flash
        read_flash.c
        spi_flash.c
        )

# pull in common dependencies
target_link_libraries(ads pico_stdlib pico_multicore hardware_i2c hardware_spi hardware_pwm ${Eigen_LIBRARIES})
target_include_directories(ads PUBLIC ../include)

target_link_libraries(read_flash pico_stdlib hardware_spi)
target_include_directories(read_flash PUBLIC ../include)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

pico_enable_stdio_usb(read_flash 1)
pico_enable_stdio_uart(read_flash 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ads)
pico_add_extra_outputs(read_flash)

