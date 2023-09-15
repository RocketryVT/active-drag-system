# Active Drag System (ADS)
This is the main codebase for Rocketry at Virginia Tech's Active Drag System, also known coloquially as the ADS, for the 2023-2024 competition year. It runs primarily on a BeagleBone Black, and its goal is to autonomously control the ADS' deployment during flight.

## BUILD
vagrant up
vagrant ssh
cmake /vagrant
cmake --build .

## RUN
scp -r src/ads debian@beaglebone.local:~/
ssh debian@beaglebone.local
./ads

## TEST
scp -r test/test_ads debian@beaglebone.local:~/
ssh debian@beaglebone.local
./test_ads

