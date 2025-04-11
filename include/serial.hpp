#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "semphr.h"

typedef struct {
    const char* name;
    const size_t len;
    void (*function)();
} command_t;


void serial_task( void *pvParameters );
int input_line(char *buffer, size_t len);

void info_cmd_func();
void help_cmd_func();
void top_cmd_func();
void clear_cmd_func();
void reset_cmd_func();

static const char* logo = \
"\n\e[0;37m#####################################################################################\n\
#                                                           \e[0;33m=                       \e[0;37m#\n\
#                                                          \e[0;33m=                        \e[0;37m#\n\
#                                                         \e[0;33m=                         \e[0;37m#\n\
#                                                                                   #\n\
#                                                      \e[0;31m#                            \e[0;37m#\n\
#                                                     \e[0;31m#                             \e[0;37m#\n\
#                                    \e[0;33m==========      \e[0;31m##                             \e[0;37m#\n\
#                             \e[0;33m==========        ===\e[0;31m###                              \e[0;37m#\n\
#                         \e[0;33m=======                 \e[0;31m### \e[0;33m=====                         \e[0;37m#\n\
#                      \e[0;33m======                    \e[0;31m###     \e[0;33m=====                      \e[0;37m#\n\
#                   \e[0;33m======                      \e[0;31m###         \e[0;33m=====                   \e[0;37m#\n\
#                 \e[0;33m======                       \e[0;31m###            \e[0;33m=====                 \e[0;37m#\n\
#               \e[0;33m======                        \e[0;31m###               \e[0;33m=====               \e[0;37m#\n\
#   \e[0;31m#########  \e[0;33m=====                        \e[0;31m####                  \e[0;33m====  \e[0;31m########   \e[0;37m #\n\
#   \e[0;31m##########                             #####                       #########    \e[0;37m#\n\
#   \e[0;31m##################################### ##### ################################    \e[0;37m#\n\
#   \e[0;31m#################################### #####  ################################    \e[0;37m#\n\
#   \e[0;31m#############   ##   ##  ##   ##   ######  ##   ##  ###  ##   ##############    \e[0;37m#\n\
#   \e[0;31m#############   ##   ##  ##   ##  ######   ##   ##  ###  ##   ##############    \e[0;37m#\n\
#   \e[0;31m#############   ##   ##  ##   ##  #####    ##   ##  ###  ##   ##############    \e[0;37m#\n\
#   \e[0;31m###############################  ###### ####################################    \e[0;37m#\n\
#   \e[0;31m##############################  #######     ################################    \e[0;37m#\n\
#   \e[0;31m###################            #######                  ####################    \e[0;37m#\n\
#   \e[0;31m#############                 ########                        ##############    \e[0;37m#\n\
#   \e[0;31m############                 #########                        ##############    \e[0;37m#\n\
#   \e[0;31m############                ###########                       ##############    \e[0;37m#\n\
#   \e[0;31m############               #############                      ##############    \e[0;37m#\n\
#   \e[0;31m############             ##################                   ##############    \e[0;37m#\n\
#   \e[0;31m############          #########################               ##############    \e[0;37m#\n\
#                                                                                   \e[0;37m#\n\
#####################################################################################\n";

#define NUM_BASE_CMDS 5
const command_t base_commands[] = { {.name = "help",
                                     .len = 4,
                                     .function = &help_cmd_func},
                                    {.name = "info",
                                     .len = 4,
                                     .function = &info_cmd_func},
                                    {.name = "clear",
                                     .len = 5,
                                     .function = &clear_cmd_func},
                                    {.name = "top",
                                     .len = 3,
                                     .function = &top_cmd_func},
                                    {.name = "reset",
                                     .len = 5,
                                     .function = &reset_cmd_func}};

extern const size_t num_user_cmds;
extern const command_t user_commands[];
