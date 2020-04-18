/*
 * shell.h
 *
 *  Created on: 15 апр. 2020 г.
 *      Author: ilya_000
 */

#ifndef COMPONENTS_S2LP_CONSOLE_SHELL_H_
#define COMPONENTS_S2LP_CONSOLE_SHELL_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"


#define BUF_LEN 256
#define RING_BUF_LEN 1024


#define send_prompt()   {send_chars((char*)prompt);}
#define send_error()   {send_chars((char*)err);}
#define send_exit()   {send_chars((char*)ex);}

typedef struct exchange_par
{
	int fd;
	RingbufHandle_t rb;
	SemaphoreHandle_t xSemaphore;
} exchange_par_t;


void start_x_shell(void);
void EUSART1_init(int console_fd);
void taskRead(void* param);


#ifdef	__cplusplus
}
#endif

#endif /* COMPONENTS_S2LP_CONSOLE_SHELL_H_ */
