# THIS IS THE BRANCH USED FOR SOUTH CAROLINA TEST LAUNCH 02/24/2024

# Active Drag System (ADS)
This is the main codebase for Rocketry at Virginia Tech's Active Drag System, also known colloquially as the ADS, for the 2023-2024 competition year. It runs primarily on a BeagleBone Black, and its goal is to autonomously control the ADS' deployment during flight.

`Eigen` Library, `cmake` and `Google Test` required for successful build.

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
sudo apt install build-essential cmake valgrind crossbuild-essential-armhf
sudo apt install musl musl-tools
```
Then to actually build:
```shell
cmake -B build
cmake --build build/
```

## BUILD Alternative (Mac)
```shell
brew install arm-linux-gnueabihf-binutils
# This installs just the arm library, you can remove x86_64 if you want those too
brew install FiloSottile/musl-cross/musl-cross --without-x86_64 --with-arm-hf
```
To check if installed correctly run:
```shell
ls /opt/homebrew/bin | grep "musl"
```
you should see:
```shell
arm-linux-musleabihf-gcc
arm-linux-musleabihf-g++
```

## RUN
```shell
scp -r src/ads debian@beaglebone.local:~/
ssh debian@beaglebone.local
./ads
```

## TEST
```shell
scp -r test/test_ads debian@beaglebone.local:~/
ssh debian@beaglebone.local
./test_ads
```

## GPIO Pins
The GPIO number is calculated by taking the GPIO chip
number, multiplying it by 32, and then adding the offset. For example,
GPIO1_12 = (1 × 32) + 12 = GPIO 44.

```shell
/sys/class/gpio/gpio44 = GPIO1_12
```
