#pragma once

#include "hardware/gpio.h"
#include <stdio.h>
#include "pico/time.h"

#define HEART_RATE_HZ 5

void heartbeat_initialize(int gpio);
