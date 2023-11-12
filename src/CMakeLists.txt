add_executable(ads
    active_drag_system.cpp
)

add_executable(pru1
        pru1/sensors.cpp
)

add_executable(pru2
        pru2/servos.cpp
)

target_link_options(pru1 PRIVATE -static)
target_link_options(pru2 PRIVATE -static)
