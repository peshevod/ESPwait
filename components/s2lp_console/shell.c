/*
 * shell.c
 *
 *  Created on: 15 ���. 2020 �.
 *      Author: ilya_000
 */

#include "shell.h"
//#include "S2LP_Config.h"
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "cmd_nvs.h"

#include "esp_vfs.h"
#include "sys/unistd.h"
#include "mbedtls/md5.h"

char t[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

char c_buf[BUF_LEN], val_buf[BUF_LEN];
uint8_t c_len;
uint8_t hex=0;
char prompt[] = {"ESPWait> "};
char err[] = {"Error\nESPWait> "};
char ex[] = {"Exit\n"};
char commands[] = {'S', 'L', 'D'};
char ver[]={"=== S2-LP shell v 1.1.5 ===\n"};

int len=0;
int gError=0;
static exchange_par_t read_par;
extern _param _params[];
static int volatile console_fd;
static int w_ready=1;
static int c_crlf=1;
static char bufr[BUF_LEN];
static int lenr=0;
static int bufr_len=0;
static char c_prev=0;
static int do_prev=0;
static int did_prev=0;
uint8_t stop_console[2];

//static int c_next;



void EUSART1_init(int fd, console_type con)
{
	console_fd=fd;
	read_par.rb=xRingbufferCreate(RING_BUF_LEN,RINGBUF_TYPE_BYTEBUF);
	read_par.fd=console_fd;
    read_par.xSemaphore = xSemaphoreCreateMutex();
	gError=0;
	w_ready=1;
	c_crlf=1;
	lenr=0;
	bufr_len=0;
	c_prev=0;
	do_prev=0;
	did_prev=0;
    xTaskCreate(taskRead, "taskRead", 2048, (void *)(&read_par), 5, NULL);
}

void taskRead(void* param)
{
	UBaseType_t uxFree;
	UBaseType_t uxRead;
	UBaseType_t uxWrite;
	UBaseType_t uxAcquire;
	UBaseType_t uxItemsWaiting;
	int size;
	char buf0[BUF_LEN];
	while(!gError)
	{
		xSemaphoreTake(((exchange_par_t*)param)->xSemaphore,portMAX_DELAY);
		vRingbufferGetInfo(read_par.rb, &uxFree, &uxRead, &uxWrite, &uxAcquire, &uxItemsWaiting);
		xSemaphoreGive(((exchange_par_t*)param)->xSemaphore);
		int l=BUF_LEN<RING_BUF_LEN-uxItemsWaiting ? BUF_LEN : RING_BUF_LEN-uxItemsWaiting;
		int k=120000;
		while(console_fd==-1 && k-->0) vTaskDelay(10 / portTICK_PERIOD_MS);
		if(console_fd!=-1)
		{
			if(l>1) size=read(console_fd,buf0,l-1);
			else size=0;
		} else size=-1;
		if(size>0)
		{
			xSemaphoreTake(((exchange_par_t*)param)->xSemaphore,portMAX_DELAY);
			if(xRingbufferSend(((exchange_par_t*)param)->rb, buf0, size, 0)==pdFALSE)
			{
				xSemaphoreGive(((exchange_par_t*)param)->xSemaphore);
				gError=11;
				break;
			}
			xSemaphoreGive(((exchange_par_t*)param)->xSemaphore);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}



char EUSART1_Read()
{
	size_t size;
	char c;
	c=-1;
	if(c_crlf && do_prev)
	{
		do_prev=0;
		did_prev=1;
		return '\n';
	}
	while(!gError)
	{
		while(!gError)
		{
			if(bufr_len!=0)
			{
				c=bufr[lenr++];
				if(lenr>=bufr_len) bufr_len=0;
				break;
			}
			xSemaphoreTake(read_par.xSemaphore,portMAX_DELAY);
			char* buf1=(char*)xRingbufferReceiveUpTo(read_par.rb, &size,0, BUF_LEN);
			if(buf1!=NULL)
			{
				memcpy(bufr,buf1,size);
				bufr_len=size;
				lenr=0;
				vRingbufferReturnItem(read_par.rb, buf1);
			}
			xSemaphoreGive(read_par.xSemaphore);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		if(!(c_crlf && c=='\n' && did_prev)) break;
		did_prev=0;
	}
	did_prev=0;
	if(c_crlf && (c=='\n' || c=='\r'))
	{
		c='\r';
		do_prev=1;
		did_prev=0;
	}
	c_prev=c;
	return c;
}

int EUSART1_is_rx_ready()
{
	UBaseType_t uxFree;
	UBaseType_t uxRead;
	UBaseType_t uxWrite;
	UBaseType_t uxAcquire;
	UBaseType_t uxItemsWaiting;
	if(bufr_len!=0 || do_prev) return 1;
	xSemaphoreTake(read_par.xSemaphore,portMAX_DELAY);
	vRingbufferGetInfo(read_par.rb, &uxFree, &uxRead, &uxWrite, &uxAcquire, &uxItemsWaiting);
	xSemaphoreGive(read_par.xSemaphore);
	if(uxItemsWaiting>0) return 1;
	else return 0;
}

static int w_did=0;

static int c_write(char* c)
{
	int k=120000, res;
	do
	{
		if(console_fd!=-1) res=write(console_fd,c,1);
		else res=-1;
		if(res!=1)
		{
			w_ready=0;
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		else
		{
			w_ready=1;
			return 1;
		}
	} while (k-->0);
	w_ready=1;
	return res;
}

int EUSART1_Write(char c)
{
	char w_c=c;
	int rc;
	if(c=='\n' || c=='\r')
	{
		if(c=='\n' && w_did)
		{
			w_did=0;
			return 1;
		}
		w_c='\r';
		rc=c_write(&w_c);
		w_c='\n';
		w_did=1;
	} else w_did=0;
	rc=c_write(&w_c);
	return rc;
}

int EUSART1_is_tx_done()
{
	if(w_ready && console_fd!=-1) return 1;
	else return 0;
}


void send_chars(char* x) {
    uint8_t i=0;
    while(x[i]!=0) EUSART1_Write(x[i++]);
    while (!EUSART1_is_tx_done());
}


void empty_RXbuffer() {
    while (EUSART1_is_rx_ready()) EUSART1_Read();
}

static int volatile timer4_expired=0;
char* data;

static void Timer4Callback( TimerHandle_t pxTimer )
{
	timer4_expired=1;
}


static int getcert(char* cert)
{
	int len=0;
	char c;
	unsigned char md5[16];
	char str_md5[40];
	char str[64];
	char key_name[16];
	if(!strcmp(cert,"KEY") || !strcmp(cert,"ROOT") || !strcmp(cert,"CERT") )
	{
		char* data=(char*)malloc(4096);
		send_chars("Sent certificate or key, at the end please press Ctrl-Z...\n");
		TimerHandle_t Timer4=xTimerCreate("Timer4",300000/portTICK_RATE_MS,pdFALSE,NULL,Timer4Callback);
		timer4_expired=0;
		xTimerStart(Timer4,0);
		gError=0;
		c_crlf=0;
		do
		{
			while(!timer4_expired && !EUSART1_is_rx_ready()) vTaskDelay(1/portTICK_RATE_MS);
			c=EUSART1_Read();
			if(c=='\xff') continue;
			if(c==0x1a) break;
//			if(c=='\n') EUSART1_Write('!');
//			if(c=='\r') EUSART1_Write('?');
			EUSART1_Write(c);
			data[len++]=c;
		} while(!timer4_expired);
		c_crlf=1;
		if(timer4_expired)
		{
			free(data);
			return 2;
		}
		if(mbedtls_md5_ret((const unsigned char*)data,len,md5))
		{
			send_chars("Error while calculating MD5\n");
			free(data);
			return 2;
		}
		sprintf(str_md5,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",md5[0],md5[1],md5[2],md5[3],md5[4],md5[5],md5[6],md5[7],md5[8],md5[9],md5[10],md5[11],md5[12],md5[13],md5[14],md5[15]);
		sprintf(str,"\nCheck MD5 sum: %s\n",str_md5);
		send_chars(str);
		strcpy(key_name,"MD5");
		strcat(key_name,cert);
		if(set_cert(cert,data,len)==ESP_OK)
		{
			ESP_LOGI(__func__,"certificate type=%s length %d successfully written to flash",cert,len);
			if(set_value_in_nvs(key_name,str_md5)==ESP_OK)
			{
				free(data);
				return 1;
			}
		}
		free(data);
	}
	return 2;
}



void print_par(char* p)
{
	char value[64];
	char y[256];
	_param* __params=_params;
    while(__params->type)
    {
        if(!strcmp(__params->c,p))
        {
        	get_value_from_nvs(p, hex, y, value);
        	send_chars(y);
            return;
        }
        __params++;
    }
}


void EUSART1_list()
{
	char value[64];
	char y[256];
	int rc;
	list_init();
	do
	{
		rc=list(hex,y,value);
		send_chars(y);
	} while(rc==1);
}


uint8_t proceed() {
    uint8_t i = 0, cmd, j;
    char par[16];
    //    printf("proceed %s\r\n",c_buf);
    c_buf[c_len] = 0;
    cmd = c_buf[i++];
    if(cmd==0) return 1;
    if(c_buf[1]=='X')
    {
        hex=1;
        i++;
    }
    else hex=0;
    if (cmd == 'Q' && c_buf[i] == 0) {
        send_exit();
        return 0;
    }
    if (cmd == 'L' && c_buf[i] == 0) {
//        print_pars();
    	EUSART1_list();
        return 1;
    }
    while (c_buf[i] == ' ' || c_buf[i] == '\t') i++;
    j=0;
    while (c_buf[i] != ' ' && c_buf[i]!='=' && c_buf[i] != 0)
    {
    	par[j++] = c_buf[i++];
    }
    par[j]=0;
    uint8_t ip = 0, ip0 = 0xff;
    do {
        if (strcmp(_params[ip].c,par)==0)
        {
            ip0 = ip;
            break;
        }
    } while (_params[++ip].type);
    if (ip0 == 0xff) return 2;
    if (cmd == 'D')
    {
        print_par(par);
        return 1;
    }
    if(cmd=='G')
    {
    	return getcert(par);
    }
    if(cmd!='S') return 2;
//    i++;
    while (c_buf[i] == ' ' || c_buf[i] == '\t') i++;
    if (c_buf[i++] != '=') return 2;
    while (c_buf[i] == ' ' || c_buf[i] == '\t') i++;
    ip = 0;
    do {
        val_buf[ip++] = c_buf[i];
    } while (c_buf[i++]);
    val_buf[ip]=0;
    set_value_in_nvs(par, val_buf);
    print_par(par);
    return 1;
}

static int volatile s2lp_console_timer_expired=0;

static void vTimerCallback( TimerHandle_t pxTimer )
{
	s2lp_console_timer_expired=1;
}


void start_x_shell(void) {
    char c;
    uint8_t start = 0;
    c_len = 0;
    add_uid();
	send_chars("\n Press any key to start console...\n");
	TimerHandle_t Timer3=xTimerCreate("Timer3",11000/portTICK_RATE_MS,pdFALSE,NULL,vTimerCallback);
    xTimerStart(Timer3,0);
    s2lp_console_timer_expired=0;
    send_chars(ver);
    send_prompt();
    while (1)
    {
        if (!start && s2lp_console_timer_expired)
        {
            send_exit();
            return;
        }
        if (EUSART1_is_rx_ready())
        {
            c = EUSART1_Read();
            EUSART1_Write(c);
            if (c == 0x08) {
                EUSART1_Write(' ');
                EUSART1_Write(c);
                c_len--;
                while (!EUSART1_is_tx_done());
                continue;
            }
            while (!EUSART1_is_tx_done());
            start = 1;
            switch (c) {
                case '\r':
                    c_buf[c_len] = 0;
                    empty_RXbuffer();
                    uint8_t r = proceed();
                    if (r == 0) return;
                    if (r != 1) send_error()
                    else send_prompt();
                    break;
                default:
                    if (c >= 0x61 && c <= 0x7A) c -= 0x20;
                    c_buf[c_len++] = c;
                    continue;
            }
            empty_RXbuffer();
            c_len = 0;
        }
    }
}



