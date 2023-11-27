cmake_minimum_required(VERSION 3.16.3)

project(ADS)

# Set Variables
set(TARGET_A kalmanFilterTest)
set(SOURCES_A ../tests/kalmanFilterTest.cpp kalmanfilter.cpp)

set(TARGET_B surfaceFitModelTest)
set(SOURCES_B ../tests/surfaceFitModelTest.cpp surfaceFitModel.cpp)

set(TARGET_C actuationPlanTest)
set(SOURCES_C ../tests/actuationPlanTest.cpp actuationPlan.cpp surfaceFitModel.cpp rocketUtils.cpp sensorIMU.cpp sensorAltimeter.cpp)

set(TARGET_D adsTest)
set(SOURCES_D ../tests/adsTest.cpp ads.cpp actuationPlan.cpp surfaceFitModel.cpp rocketUtils.cpp sensorIMU.cpp sensorAltimeter.cpp motor.cpp logger.cpp kalmanfilter.cpp)

set(TARGET_E motorPlanTest)
set(SOURCES_E ../tests/motorTest.cpp motor.cpp rocketUtils.cpp)

set(TARGET_F sensorAltimeterPlanTest)
set(SOURCES_F ../tests/sensorAltimeterTest.cpp sensorAltimeter.cpp rocketUtils.cpp)

set(TARGET_G sensorIMUPlanTest)
set(SOURCES_G ../tests/sensorIMUTest.cpp sensorIMU.cpp rocketUtils.cpp)


#Set-up Google Test
set(CMAKE_CXX_STANDARD 14)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)


# Create Executables & Link Dependencies
add_executable(${TARGET_A} ${SOURCES_A})
target_link_libraries(${TARGET_A} PUBLIC gtest_main) # Link GoogleTest's main() to Executable
# Tell CMake the Target is a Unit Test
add_test(NAME ${TARGET_A} COMMAND ${TARGET_A})


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