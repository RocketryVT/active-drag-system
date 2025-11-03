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

add_executable(rtos_blinky
                 rtos_blinky.c
)
pico_set_binary_type(ads copy_to_ram)

target_link_libraries(ads pico_logger pico_stdlib pico_stdio pico_multicore pico_sync hardware_i2c hardware_pwm hardware_adc ${Eigen_LIBRARIES})
target_include_directories(ads PUBLIC ../include)

target_include_directories(rtos_blinky PUBLIC ${CMAKE_CURRENT_LIST_DIR} ../include)
target_compile_definitions(rtos_blinky PRIVATE
        PICO_STDIO_STACK_BUFFER_SIZE=64 # use a small printf on stack buffer
)
target_link_libraries(rtos_blinky pico_stdlib pico_stdio pico_multicore hardware_i2c pico_sync FreeRTOS-Kernel FreeRTOS-Kernel-Heap4)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

pico_add_extra_outputs(ads)
pico_add_extra_outputs(rtos_blinky)

