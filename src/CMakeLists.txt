add_executable(ads
               active_drag_system.cpp
               heartbeat.cpp
               pwm.cpp
               altimeter.cpp
               mid_imu.cpp
               high_accel.cpp
               magnetometer.cpp
               sensor_i2c.cpp
)

pico_set_binary_type(ads copy_to_ram)

target_link_libraries(ads pico_logger pico_stdlib pico_stdio pico_multicore pico_sync hardware_i2c hardware_pwm hardware_adc ${Eigen_LIBRARIES})
target_include_directories(ads PUBLIC ../include)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

pico_add_extra_outputs(ads)

