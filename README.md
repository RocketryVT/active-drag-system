# Active Drag System (ADS)
This is the main codebase for Rocketry at Virginia Tech's Active Drag System, also known coloquially as the ADS, for the 2023-2024 competition year. It runs primarily on a BeagleBone Black, and its goal is to autonomously control the ADS' deployment during flight.

`Eigen` Library, `cmake` and `Google Test` required for successful build.

## BUILD
cmake -B build
cmake --build build/

## RUN
cd build && src/ads

## TEST
cd build && ctest

