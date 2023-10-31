add_executable(pru1
        pru1/Sensors.cpp
        pru1/SensorLogger.cpp
        pru1/SensorStructs.h
        pru1/AsyncSerial.h
)

add_executable(pru2
        pru2/Servos.cpp
        pru2/ServoLogger.cpp
        pru2/ServoStructs.h
)

set(BOOST_INCLUDE_LIBRARIES asio)
set(BOOST_ENABLE_CMAKE ON)
find_package(Boost)
target_link_libraries(pru1 ${Boost_LIBRARIES})
target_link_libraries(pru2 ${Boost_LIBRARIES})

target_link_options(pru1 PRIVATE -static)
target_link_options(pru2 PRIVATE -static)