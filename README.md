# Active Drag System (ADS)
This is the main codebase for Rocketry at Virginia Tech's Active Drag System, also known colloquially as the ADS, for the 2023-2024 competition year. It runs primarily on a Raspberry Pi Pico, and its goal is to autonomously control the ADS' deployment during flight.

`Eigen` Library, `cmake`, and `arm-none-eabi-gcc` tooling required for successful build.

## Clone

**Note: You must initialize the git submodules prior to utilizing CMake for a proper build.**

```shell
git clone https://github.com/RocketryVT/active-drag-system.git
cd active-drag-system/
git submodule update --init --recursive
```

## Build (Linux)
```shell
cmake -B build
cmake --build build
```
In the event that your preferred IDE has trouble locating header files and/or is displaying incorrect errors, pass ```-DCMAKE_EXPORT_COMPILE_COMMANDS=true``` to the first CMake command above. Similarly, if you wish to compile the additional tools (e.g. reading flash, calibrating the IMU, etc.), pass ```-DCOMPILE_TOOLS=true``` to the first CMake command above as well.

## Build (Windows)
Enable WSL2 in windows
Install Ubuntu 22 LTS from Windows Store
```shell
sudo apt update && upgrade
sudo apt install build-essential cmake gcc-arm-none-eabi
```
Then to actually build:
```shell
cmake -B build
cmake --build build
```

## Build Alternative (Mac)
```shell
brew install arm-none-eabi-gcc
```
To check if installed correctly run:
```shell
ls /opt/homebrew/bin | grep "none"
```
you should see a list including but not limited to:
```shell
arm-none-eabi-gcc
arm-none-eabi-g++
```
Next:
```shell
cmake -B build
cmake --build build
```

Binary files should be located in build/src/*.uf2 after a successful build.
