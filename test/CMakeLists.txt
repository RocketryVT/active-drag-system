if (COMPILE_TESTS)
	# create executable targets for compilation
	add_executable(alt_test
					alt_test.cpp
	)

	add_executable(clock_gen_test
					clock_gen_test.cpp
	)

	add_executable(peripheral_validation_test
					peripheral_validation_test.cpp
	)

	add_executable(quaternion_test
					quaternion_test.cpp
	)

	add_executable(servo_test
					servo_test.cpp
	)

	# pull in common dependencies
	target_link_libraries(alt_test pico_stdlib hardware_i2c hardware_gpio)

	target_link_libraries(clock_gen_test pico_stdlib hardware_clocks hardware_gpio)
	
	target_link_libraries(peripheral_validation_test 
		hardware_adc
		hardware_clocks
		hardware_gpio
		hardware_i2c
		pico_platform
		pico_stdlib
		pico_time
	)

	target_link_libraries(quaternion_test pico_stdlib hardware_i2c)
	target_include_directories(quaternion_test PUBLIC ../include)

	target_link_libraries(servo_test pico_stdlib hardware_pwm hardware_i2c)
	target_include_directories(servo_test PUBLIC ../include)

	# enable USB debugging output over serial
	pico_enable_stdio_usb(alt_test 1)
	pico_enable_stdio_uart(alt_test 0)

	pico_enable_stdio_usb(clock_gen_test 1)
	pico_enable_stdio_uart(clock_gen_test 0)

	pico_enable_stdio_usb(peripheral_validation_test 1)
	pico_enable_stdio_uart(peripheral_validation_test 0)

	pico_enable_stdio_usb(quaternion_test 1)
	pico_enable_stdio_uart(quaternion_test 0)

	pico_enable_stdio_usb(servo_test 1)
	pico_enable_stdio_uart(servo_test 0)

	# create map/bin/hex file etc.
	pico_add_extra_outputs(alt_test)
	pico_add_extra_outputs(clock_gen_test)
	pico_add_extra_outputs(peripheral_validation_test)
	pico_add_extra_outputs(quaternion_test)
	pico_add_extra_outputs(servo_test)
endif()
