# Active Drag System (ADS)
This is the main codebase for Rocketry at Virginia Tech's Active Drag System, also known colloquially as the ADS, for the 2023-2024 competition year. It runs primarily on a Raspberry Pi Pico, and its goal is to autonomously control the ADS' deployment during flight.

`Eigen` Library, `cmake`, and `arm-none-eabi-gcc` tooling required for successful build.
## Before Build
```shell
git clone https://github.com/RocketryVT/active-drag-system.git
cd active-drag-system/
git submodule update --init --recursive
```

## BUILD
```shell
vagrant up
vagrant ssh
cmake /vagrant
cmake --build .
```

## BUILD Alternative (Windows)
Enable WSL2 in windows
Install Ubuntu 22 LTS from Windows Store
```shell
sudo apt update && upgrade
sudo apt install build-essential cmake valgrind gcc-arm-none-eabi
```
Then to actually build:
```shell
cmake -B build
cmake --build build
```

## BUILD Alternative (Mac)
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

Binary files should be located in build/src/*.elf after a successful build.
