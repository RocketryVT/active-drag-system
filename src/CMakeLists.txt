add_executable(ads
        active_drag_system.cpp
        )

# pull in common dependencies
target_link_libraries(ads pico_stdlib pico_cyw43_arch_none)

# create map/bin/hex file etc.
pico_add_extra_outputs(ads)

