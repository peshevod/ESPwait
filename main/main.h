/*
 * main.h
 *
 *  Created on: 6 но€б. 2019 г.
 *      Author: ilya
 */
#include "esp_event.h"

#ifndef MAIN_MAIN_H_
#define MAIN_MAIN_H_

#define UART0_TXD  (UART_PIN_NO_CHANGE)
#define UART0_RXD  (UART_PIN_NO_CHANGE)
#define UART0_RTS  (UART_PIN_NO_CHANGE)
#define UART0_CTS  (UART_PIN_NO_CHANGE)

#define UART2_TXD  (GPIO_NUM_22)
#define UART2_RXD  (GPIO_NUM_19)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (256)
#define MAX_MESSAGE_SIZE (64)
#define MAX_MESSAGES_IN_QUEUE (16)
#define MAX_REC_SIZE 128

#define ESP_MAXIMUM_RETRY (3)

#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0)

void init_uart0();
void init_uart2();
static void uart_rec_task(void *arg);
void send_to_cloud(char* mes);
static void queue_watch_task(void *arg);
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void IRAM_ATTR s2lp_intr_handler(void* arg);
static void s2lp_wait(void *arg);

#endif /* MAIN_MAIN_H_ */
