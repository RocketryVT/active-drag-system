add_executable(active_drag_system
               ${PROJECT_SOURCE_DIR}/src/active_drag_system.cpp
               ${PROJECT_SOURCE_DIR}/src/ms5607.cpp
               ${PROJECT_SOURCE_DIR}/src/adxl375.cpp
               ${PROJECT_SOURCE_DIR}/src/iim42653.cpp
               ${PROJECT_SOURCE_DIR}/src/mmc5983ma.cpp
               ${PROJECT_SOURCE_DIR}/src/pwm.cpp
               ${PROJECT_SOURCE_DIR}/src/log_format.cpp
               ${PROJECT_SOURCE_DIR}/src/heartbeat.cpp
               ${PROJECT_SOURCE_DIR}/src/serial.cpp
)

pico_set_binary_type(active_drag_system copy_to_ram)

target_link_libraries(active_drag_system pico_stdlib pico_logger pico_flash pico_rand pico_multicore pico_sync hardware_i2c hardware_adc hardware_timer hardware_pwm FreeRTOS-Kernel FreeRTOS-Kernel-Heap4 libfixmath libfixmatrix)
target_include_directories(active_drag_system PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_compile_definitions(active_drag_system PRIVATE
        USE_FREERTOS=1
        DEBUG=1
        PICO_STDIO_STACK_BUFFER_SIZE=64 # use a small printf on stack buffer
)

pico_enable_stdio_usb(active_drag_system 1)
pico_enable_stdio_uart(active_drag_system 0)

pico_add_extra_outputs(active_drag_system)

