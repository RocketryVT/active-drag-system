add_executable(ads
               active_drag_system.cpp
               spi_flash.c
               imu.cpp
               pwm.cpp
               altimeter.cpp
               kalman_filter.cpp
)

target_link_libraries(ads pico_stdlib pico_multicore pico_sync hardware_i2c hardware_spi hardware_pwm hardware_adc pico_cyw43_arch_none ${Eigen_LIBRARIES})
target_include_directories(ads PUBLIC ../include)

pico_enable_stdio_usb(ads 0)
pico_enable_stdio_uart(ads 0)

pico_add_extra_outputs(ads)

