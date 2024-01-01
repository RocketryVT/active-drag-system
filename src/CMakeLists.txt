cmake_minimum_required(VERSION 3.16.3)

project(ADS)

include_directories( ../include )

# Set Variables
set(TARGET ActiveDragSystem)
set(SOURCES active-drag-system.cpp ads.cpp actuationPlan.cpp surfaceFitModel.cpp rocketUtils.cpp sensorIMU.cpp sensorAltimeter.cpp motor.cpp logger.cpp kalmanfilter.cpp)


# Create Executables & Link Dependencies
add_executable(${TARGET} ${SOURCES})

