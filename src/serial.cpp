#include "serial.hpp"
#include "portmacro.h"
#include <pico/multicore.h>
#include <pico/stdio.h>

void serial_task( void *pvParameters ) {
    char *str = (char *) malloc(1024);
    size_t len = 1024;

    
    printf("Welcome! :3\n");

    while (1) {
        printf("# ");
        stdio_flush();

        size_t n = input_line(str,len);
        printf("\n");
        if (n > 0) {
            for (int i = 0; i < (NUM_BASE_CMDS + num_user_cmds); i++) {
                if ((i < NUM_BASE_CMDS) && n >= base_commands[i].len && strncmp(str, base_commands[i].name, base_commands[i].len) == 0) {
                    base_commands[i].function();
                    break;
                }

                if ((i >= NUM_BASE_CMDS && ((i - NUM_BASE_CMDS) < num_user_cmds)) && n >= user_commands[i - NUM_BASE_CMDS].len && strncmp(str, user_commands[i - NUM_BASE_CMDS].name, user_commands[i - NUM_BASE_CMDS].len) == 0) {
                    user_commands[i - NUM_BASE_CMDS].function();
                    break;
                }
                if (i == (NUM_BASE_CMDS + num_user_cmds - 1)) {
                    printf("Invalid command! Please try again or use 'help' to see the available commands.\n");
                }
            }
        }
    }
}

int input_line(char *buffer, size_t len) {
    size_t n = 0;
    int c;
    for (n = 0; n < len - 1; n++) {

        c = getchar_timeout_us(0);
        switch (c) {
            case 127: /* fall through to below */
            case '\b': /* backspace character received */
                if (n > 0)
                    n--;
                buffer[n] = 0;
                stdio_putchar('\b'); /* output backspace character */
                stdio_putchar(' ');
                stdio_putchar('\b');
                n--; /* set up next iteration to deal with preceding char location */
                break;
            case '\n': /* fall through to \r */
            case '\r':
                buffer[n] = 0;
                return n;
            default:
                if (c != PICO_ERROR_TIMEOUT && c < 256) {
                    stdio_putchar(c);
                    buffer[n] = c;
                } else {
                    n--;
                }
            break;
        }
    }
    buffer[len - 1] = 0;
    return 0; // Filled up buffer without reading a linebreak
}

void info_cmd_func() {
    extern const char* executeable_name;
    printf("%s", logo);
    printf("#####################################################################################\n");
    printf("#                          \e[0;31mRocketry \e[0;37mat \e[0;33mVirginia Tech\e[0;37m                                #\n");
    printf("#                         Executeable: %s                       #\n", executeable_name);
    printf("#####################################################################################\n\n");
}

void help_cmd_func() {
    printf("Commands: \n");
    for (int i = 0; i < NUM_BASE_CMDS; i++) {
        printf("\t%s\n", base_commands[i].name);
    }

    for (int i = 0; i < num_user_cmds; i++) {
        printf("\t%s\n", user_commands[i].name);
    }
}

void clear_cmd_func() {
    printf("%c%c%c%c",0x1B,0x5B,0x32,0x4A);
}

void top_cmd_func() {
#if (configGENERATE_RUN_TIME_STATS == 1)
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    char* buffer = (char *) malloc( 40 * num_tasks );
    vTaskGetRunTimeStats(buffer);
    printf("%s", buffer);
    free(buffer);
#endif
}

static void reset_cmd_task(void * unused_arg) {
    reset_cmd_func();
}

void reset_cmd_func() {
#define AIRCR_Register (*((volatile uint32_t*)(PPB_BASE + 0x0ED0C)))
    if (get_core_num() == 0) {
        vTaskSuspendAll();
        multicore_reset_core1();
        printf("\nOn core zero! Going dark!\n");
        stdio_flush();
        AIRCR_Register = 0x5FA0004;
    } else {
        printf("\nOn core one! Tasking core zero with the reset!\n");
        stdio_flush();
        TaskHandle_t reset_task = NULL;
        vTaskSuspendAll();
        xTaskCreate(reset_cmd_task, "reset", 256, NULL, (configMAX_PRIORITIES - 1), &reset_task);
        vTaskCoreAffinitySet( reset_task, 0x01 );
        xTaskResumeAll();
    }
}
