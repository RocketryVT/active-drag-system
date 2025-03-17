#add_executable(ads
#               active_drag_system.cpp
#               spi_flash.c
#               imu.cpp
#               pwm.cpp
#               altimeter.cpp
#               kalman_filter.cpp
#)
#
#target_link_libraries(ads pico_stdlib pico_multicore pico_sync hardware_i2c hardware_spi hardware_pwm hardware_adc pico_cyw43_arch_none ${Eigen_LIBRARIES})
#target_include_directories(ads PUBLIC ../include)
#
#pico_enable_stdio_usb(ads 0)
#pico_enable_stdio_uart(ads 0)
#
#pico_add_extra_outputs(ads)

add_executable(validatePeripherals
	peripheral_validation_test.cpp
	mid_imu.cpp
	high_accel.cpp
	altimeter.cpp
	magnetometer.cpp
	sensor_i2c.cpp
)

target_link_libraries(validatePeripherals
	pico_platform
	pico_stdlib
	pico_time
	hardware_gpio
	hardware_i2c
	hardware_adc
	hardware_clocks
)

target_include_directories(validatePeripherals PUBLIC ../include)

pico_enable_stdio_usb(validatePeripherals 1)
pico_enable_stdio_uart(validatePeripherals 0)

pico_add_extra_outputs(validatePeripherals)
	
