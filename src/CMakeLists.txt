add_executable(pru1
        pru1/sensors.cpp
        pru1/logger.cpp
)

add_executable(pru2
        pru2/servos.cpp
)

set(BOOST_INCLUDE_LIBRARIES asio)
set(BOOST_ENABLE_CMAKE ON)
find_package(Boost)
target_link_libraries(pru1 ${Boost_LIBRARIES})
target_link_libraries(pru2 ${Boost_LIBRARIES})

target_link_options(pru1 PRIVATE -static)
target_link_options(pru2 PRIVATE -static)