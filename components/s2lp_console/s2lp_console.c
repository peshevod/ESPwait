/* Console example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "linenoise/linenoise.h"
#include "esp_vfs_fat.h"
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_nvs.h"
#include "s2lp_console.h"
#include "shell.h"
#include "spp_server.h"

static const char* TAG = "s2lp_console";
extern uint8_t s2lp_console_ex;
extern int volatile console_fd;
char x[128];
extern char server_name[40];
extern uint8_t stop_console[2];
extern RTC_SLOW_ATTR uint32_t uid;

static void initialize_console()
{
    /* Disable buffering on stdin */
	setvbuf(stdin, NULL, _IONBF, 0);
    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
	esp_vfs_dev_uart_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
	esp_vfs_dev_uart_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Tell VFS to use UART driver */
	esp_vfs_dev_uart_use_driver(UART_NUM_0);

    /* Initialize the console */
    esp_console_config_t console_config = {
            .max_cmdline_args = 8,
            .max_cmdline_length = 256,
#if CONFIG_LOG_COLORS
            .hint_color = atoi(LOG_COLOR_CYAN)
#endif
    };
    ESP_ERROR_CHECK( esp_console_init(&console_config) );

    /* Configure linenoise line completion library */
    /* Enable multiline editing. If not set, long commands will scroll within
     * single line.
     */
    linenoiseSetMultiLine(1);

    /* Tell linenoise where to get command completions and hints */
    linenoiseSetCompletionCallback(&esp_console_get_completion);
    linenoiseSetHintsCallback((linenoiseHintsCallback*) &esp_console_get_hint);

    /* Set command history size */
    linenoiseHistorySetMaxLen(100);


}

int volatile s2lp_console_timer_expired;

static void vTimerCallback( TimerHandle_t pxTimer )
{
	s2lp_console_timer_expired=1;
}


void bt_console()
{
    sprintf(server_name,"ESPWAIT-%08X",uid);
    init_spp_server();
    int k=120;
    while(--k>0 && console_fd==-1)
    {
    	if(stop_console[BT_CONSOLE]) break;
    	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    if(console_fd!=-1)
    {
    	start_x_shell(BT_CONSOLE);
    }
    ESP_LOGI("bt_console","shutting SPP server\n");
    shutdown_spp_server();
    ESP_LOGI("bt_console","BT_CONSOLE stopped\n");
    stop_console[BT_CONSOLE]=2;
	vTaskDelete(NULL);
}

void serial_console()
{
   	start_x_shell(SERIAL_CONSOLE);
    ESP_LOGI("serial_console","SERIAL CONSOLE stopped\n");
    stop_console[SERIAL_CONSOLE]=2;
	vTaskDelete(NULL);
}

void start_s2lp_console()
{
    size_t len;
    get_uid(&uid);
    stop_console[SERIAL_CONSOLE]=0;
    stop_console[BT_CONSOLE]=0;
    uart_flush_input(UART_NUM_0);
    xTaskCreatePinnedToCore(bt_console, "bt_console", 8192, NULL, 10, NULL,0);
    xTaskCreatePinnedToCore(serial_console, "serial_console", 4096, NULL, 10, NULL,0);
//    vTaskStartScheduler();

    while(stop_console[SERIAL_CONSOLE]!=2 || stop_console[BT_CONSOLE]!=2) vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("\n Stopped all consoles\n");

    return;

    printf("\n Press any key to start console...\n");
    if(console_fd==-1) uart_flush_input(UART_NUM_0);
	TimerHandle_t Timer3=xTimerCreate("Timer3",11000/portTICK_RATE_MS,pdFALSE,NULL,vTimerCallback);
    xTimerStart(Timer3,0);
    s2lp_console_timer_expired=0;
    if(console_fd==-1)
    {while(!s2lp_console_timer_expired)
    {
    	uart_get_buffered_data_len(UART_NUM_0,&len);
    	if (len!=0)
    	{
    		uart_flush_input(UART_NUM_0);
    		break;
    	}
    }
    }

    if(xTimerIsTimerActive(Timer3)) xTimerStop(Timer3,0);
    while(xTimerIsTimerActive(Timer3));
    xTimerDelete(Timer3,0);
    if(s2lp_console_timer_expired) return;

	initialize_console();

    /* Register commands */
    esp_console_register_help_command();
    register_nvs();

    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    const char* prompt = LOG_COLOR_I "s2lp> " LOG_RESET_COLOR;
    /* Figure out if the terminal supports escape sequences */
    int probe_status = linenoiseProbe();
    if (probe_status) { /* zero indicates success */
        printf("\n"
               "Your terminal application does not support escape sequences.\n"
               "Line editing and history features are disabled.\n"
               "On Windows, try using Putty instead.\n");
        linenoiseSetDumbMode(1);
#if CONFIG_LOG_COLORS
        /* Since the terminal doesn't support escape sequences,
         * don't use color codes in the prompt.
         */
        prompt = "s2lp> ";
#endif //CONFIG_LOG_COLORS
    }

    /* Main loop */
    while(!s2lp_console_ex) {
        /* Get a line using linenoise.
         * The line is returned when ENTER is pressed.
         */
        char* line = linenoise(prompt);
        if (line == NULL) { /* Ignore empty lines */
            continue;
        }
        /* Add the command to the history */
        linenoiseHistoryAdd(line);
#if CONFIG_STORE_HISTORY
        /* Save command history to filesystem */
        linenoiseHistorySave(HISTORY_PATH);
#endif

//        while(1);
        /* Try to run the command */
        int ret;
        esp_err_t err = esp_console_run(line, &ret);
        if (err == ESP_ERR_NOT_FOUND) {
            printf("Unrecognized command\n");
        } else if (err == ESP_ERR_INVALID_ARG) {
            // command was empty
        } else if (err == ESP_OK && ret != ESP_OK) {
            printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(err));
        } else if (err != ESP_OK) {
            printf("Internal error: %s\n", esp_err_to_name(err));
        }
        /* linenoise allocates line buffer on the heap, so need to free it */
        linenoiseFree(line);
    }
}
