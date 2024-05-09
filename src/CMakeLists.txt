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
cmake_minimum_required(VERSION 3.16.3)

include_directories( ../include )

# Set Variables
set(TARGET ActiveDragSystem)
set(SOURCES active-drag-system.cpp ads.cpp actuationPlan.cpp surfaceFitModel.cpp rocketUtils.cpp sensorIMU.cpp sensorAltimeter.cpp motor.cpp logger.cpp kalmanfilter.cpp)


# Create Executables & Link Dependencies
add_executable(${TARGET} ${SOURCES})

add_executable(${TARGET_B} ${SOURCES_B})
target_link_libraries(${TARGET_B} PUBLIC gtest_main)
add_test(NAME ${TARGET_B} COMMAND ${TARGET_B})

add_executable(${TARGET_C} ${SOURCES_C})
target_link_libraries(${TARGET_C} PUBLIC gtest_main)
add_test(NAME ${TARGET_C} COMMAND ${TARGET_C})

add_executable(${TARGET_D} ${SOURCES_D})
target_link_libraries(${TARGET_D} PUBLIC gtest_main)
add_test(NAME ${TARGET_D} COMMAND ${TARGET_D})

add_executable(${TARGET_E} ${SOURCES_E})
target_link_libraries(${TARGET_E} PUBLIC gtest_main)
add_test(NAME ${TARGET_E} COMMAND ${TARGET_E})

add_executable(${TARGET_F} ${SOURCES_F})
target_link_libraries(${TARGET_F} PUBLIC gtest_main)
add_test(NAME ${TARGET_F} COMMAND ${TARGET_F})

add_executable(${TARGET_G} ${SOURCES_G})
target_link_libraries(${TARGET_G} PUBLIC gtest_main)
add_test(NAME ${TARGET_G} COMMAND ${TARGET_G})
