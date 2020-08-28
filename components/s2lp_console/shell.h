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
#include "s2lp_console.h"


#define BUF_LEN 1024
#define RING_BUF_LEN 1024


#define send_prompt()   {send_chars(con, (char*)prompt);}
#define send_error()   {send_chars(con, (char*)err);}
#define send_exit()   {send_chars(con, (char*)ex);}

typedef struct exchange_par
{
	RingbufHandle_t rb;
	SemaphoreHandle_t xSemaphore;
	int gError;
	char c_buf[BUF_LEN];
	char val_buf[BUF_LEN];
	uint8_t c_len;
	uint8_t hex;
	int w_ready;
	int w_did;
	int c_crlf;
	int lenr;
	char bufr[BUF_LEN];
	int bufr_len;
	char c_prev;
	int do_prev;
	int did_prev;
	int len;
	console_type con;
} exchange_par_t;


void start_x_shell(console_type con);
void EUSART1_init(console_type con);
void taskRead(void* param);
void send_chars(console_type con, char* x);

#ifdef	__cplusplus
}
#endif

#endif /* COMPONENTS_S2LP_CONSOLE_SHELL_H_ */
