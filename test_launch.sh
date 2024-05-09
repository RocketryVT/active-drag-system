#!/bin/sh

config-pin p8.12 pruout
echo 'start' > /sys/class/remoteproc/remoteproc1/state
