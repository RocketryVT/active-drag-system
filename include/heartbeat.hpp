#pragma once

#include "hardware/gpio.h"
#include <stdio.h>
#include "pico/time.h"

#define HEART_RATE_HZ 5

void heartbeat_initialize(int gpio);

#if (USE_FREERTOS == 1)
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "serial.hpp"
#include "task.h"
#include "semphr.h"

void heartbeat_task( void *pvParameters );
#endif
