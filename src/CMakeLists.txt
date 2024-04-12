add_executable(ads
        active_drag_system.cpp
        bno055.cpp
        )

# pull in common dependencies
target_link_libraries(ads pico_stdlib hardware_i2c)
target_include_directories(ads PUBLIC ../include)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ads)

