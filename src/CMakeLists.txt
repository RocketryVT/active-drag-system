add_executable(ads
        active_drag_system.cpp
        )

# pull in common dependencies
target_link_libraries(ads pico_stdlib hardware_i2c)

pico_enable_stdio_usb(ads 1)
pico_enable_stdio_uart(ads 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ads)

