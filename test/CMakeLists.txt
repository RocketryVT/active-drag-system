cmake_minimum_required(VERSION 3.16.3)

project(ADSTests)

# Set Variables
set(TARGET_A TESTkalmanFilter)
set(SOURCES_A kalmanFilterTest.cpp ../src/kalmanfilter.cpp)

set(TARGET_B TESTsurfaceFitModel)
set(SOURCES_B surfaceFitModelTest.cpp ../src/surfaceFitModel.cpp)

set(TARGET_C TESTactuationPlan)
set(SOURCES_C actuationPlanTest.cpp ../src/actuationPlan.cpp ../src/surfaceFitModel.cpp ../src/rocketUtils.cpp ../src/sensorIMU.cpp ../src/sensorAltimeter.cpp)

# set(TARGET_D TESTads)
# set(SOURCES_D adsTest.cpp ../src/ads.cpp ../src/actuationPlan.cpp ../src/surfaceFitModel.cpp ../src/rocketUtils.cpp ../src/sensorIMU.cpp ../src/sensorAltimeter.cpp ../src/motor.cpp ../src/logger.cpp ../src/kalmanfilter.cpp)

set(TARGET_E TESTmotorPlan)
set(SOURCES_E motorTest.cpp ../src/motor.cpp ../src/rocketUtils.cpp)

set(TARGET_F TESTsensorAltimeterPlan)
set(SOURCES_F sensorAltimeterTest.cpp ../src/sensorAltimeter.cpp ../src/rocketUtils.cpp)

set(TARGET_G TESTsensorIMUPlan)
set(SOURCES_G sensorIMUTest.cpp ../src/sensorIMU.cpp ../src/rocketUtils.cpp)


#Set-up Google Test
set(CMAKE_CXX_STANDARD 14)
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
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

# add_executable(${TARGET_D} ${SOURCES_D})
# target_link_libraries(${TARGET_D} PUBLIC gtest_main)
# add_test(NAME ${TARGET_D} COMMAND ${TARGET_D})

add_executable(${TARGET_E} ${SOURCES_E})
target_link_libraries(${TARGET_E} PUBLIC gtest_main)
add_test(NAME ${TARGET_E} COMMAND ${TARGET_E})

add_executable(${TARGET_F} ${SOURCES_F})
target_link_libraries(${TARGET_F} PUBLIC gtest_main)
add_test(NAME ${TARGET_F} COMMAND ${TARGET_F})

add_executable(${TARGET_G} ${SOURCES_G})
target_link_libraries(${TARGET_G} PUBLIC gtest_main)
add_test(NAME ${TARGET_G} COMMAND ${TARGET_G})