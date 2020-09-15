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
#include "driver/uart.h"

#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "sys/unistd.h"
#include "mbedtls/md5.h"

char t[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

char prompt[] = {"ESPWait> "};
char err[] = {"Error\nESPWait> "};
char ex[] = {"Exit\n"};
char commands[] = {'S', 'L', 'D'};
char ver[]={"=== S2-LP shell v 1.1.5 ===\n"};

static exchange_par_t z[2];
extern _param _params[];
extern int volatile console_fd;
uint8_t stop_console[2];
TaskHandle_t rtask[2];
//static int c_next;



void EUSART1_init(console_type con)
{
	char tname[10];
	z[con].rb=xRingbufferCreate(RING_BUF_LEN,RINGBUF_TYPE_BYTEBUF);
    z[con].xSemaphore = xSemaphoreCreateMutex();
    z[con].gError=0;
	z[con].c_len=0;
	z[con].hex=0;
	z[con].w_ready=1;
	z[con].c_crlf=1;
	z[con].lenr=0;
	z[con].bufr_len=0;
	z[con].c_prev=0;
	z[con].do_prev=0;
	z[con].did_prev=0;
	z[con].w_did=0;
	if(con) strcpy(tname,"taskRead1"); else strcpy(tname,"taskRead0");
    xTaskCreate(taskRead, tname, 2048, (void *)(con), 5, &(rtask[con]));
}

void taskRead(void* param)
{
	console_type con=(console_type)param;
	UBaseType_t uxFree;
	UBaseType_t uxRead;
	UBaseType_t uxWrite;
	UBaseType_t uxAcquire;
	UBaseType_t uxItemsWaiting;
	int size;
	char buf0[BUF_LEN];
	int l;
	int k;
	while(!z[con].gError)
	{
		if(stop_console[con]) break;
		xSemaphoreTake(z[con].xSemaphore,portMAX_DELAY);
		vRingbufferGetInfo(z[con].rb, &uxFree, &uxRead, &uxWrite, &uxAcquire, &uxItemsWaiting);
		xSemaphoreGive(z[con].xSemaphore);
		l=BUF_LEN<RING_BUF_LEN-uxItemsWaiting ? BUF_LEN : RING_BUF_LEN-uxItemsWaiting;
		k=120000;
		while((console_fd==-1 && con==BT_CONSOLE) && k-->0 && !stop_console[con])
		{
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		if(stop_console[con]) break;
		if((con==BT_CONSOLE && console_fd!=-1) || con==SERIAL_CONSOLE)
		{
			if(l>1)
			{
				size=-1;
				if(con==BT_CONSOLE) size=read(console_fd,buf0,l-1);
				if(con==SERIAL_CONSOLE) size=uart_read_bytes(UART_NUM_0,(uint8_t*)buf0,l-1,1);
			}
			else size=0;
		}
		else
		{
			stop_console[con]=1;
			break;
		}
		if(size>0)
		{
			xSemaphoreTake(z[con].xSemaphore,portMAX_DELAY);
			if(xRingbufferSend(z[con].rb, buf0, size, 0)==pdFALSE)
			{
				xSemaphoreGive(z[con].xSemaphore);
				z[con].gError=11;
				break;
			}
			xSemaphoreGive(z[con].xSemaphore);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}



char EUSART1_Read(console_type con)
{
	size_t size;
	char c;
	c=-1;
	if(z[con].c_crlf && z[con].do_prev)
	{
		z[con].do_prev=0;
		z[con].did_prev=1;
		return '\n';
	}
	while(!z[con].gError)
	{
		while(!z[con].gError)
		{
			if(z[con].bufr_len!=0)
			{
				c=z[con].bufr[z[con].lenr++];
				if(z[con].lenr>=z[con].bufr_len) z[con].bufr_len=0;
				break;
			}
			xSemaphoreTake(z[con].xSemaphore,portMAX_DELAY);
			char* buf1=(char*)xRingbufferReceiveUpTo(z[con].rb, &size,0, BUF_LEN);
			if(buf1!=NULL)
			{
				memcpy(z[con].bufr,buf1,size);
				z[con].bufr_len=size;
				z[con].lenr=0;
				vRingbufferReturnItem(z[con].rb, buf1);
			}
			xSemaphoreGive(z[con].xSemaphore);
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
		if(!(z[con].c_crlf && c=='\n' && z[con].did_prev)) break;
		z[con].did_prev=0;
	}
	z[con].did_prev=0;
	if(z[con].c_crlf && (c=='\n' || c=='\r'))
	{
		c='\r';
		z[con].do_prev=1;
		z[con].did_prev=0;
	}
	z[con].c_prev=c;
	return c;
}

int EUSART1_is_rx_ready(console_type con)
{
	UBaseType_t uxFree;
	UBaseType_t uxRead;
	UBaseType_t uxWrite;
	UBaseType_t uxAcquire;
	UBaseType_t uxItemsWaiting;
	if(z[con].bufr_len!=0 || z[con].do_prev) return 1;
	xSemaphoreTake(z[con].xSemaphore,portMAX_DELAY);
	vRingbufferGetInfo(z[con].rb, &uxFree, &uxRead, &uxWrite, &uxAcquire, &uxItemsWaiting);
	xSemaphoreGive(z[con].xSemaphore);
	if(uxItemsWaiting>0) return 1;
	else return 0;
}

static int c_write(console_type con, char* c)
{
	int k=120000, res;
	do
	{
		if((con==BT_CONSOLE && console_fd!=-1) || con==SERIAL_CONSOLE)
		{
			res=-1;
			if(con==BT_CONSOLE) res=write(console_fd,c,1);
			if(con==SERIAL_CONSOLE) res=uart_write_bytes(UART_NUM_0,c,1);
		}
		else
		{
			stop_console[con]=1;
			res=-1;
			break;
		}
		if(res!=1)
		{
			z[con].w_ready=0;
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		else
		{
			z[con].w_ready=1;
			return 1;
		}
		if(con==BT_CONSOLE) send_chars(SERIAL_CONSOLE,".");
	} while (k-->0);
	z[con].w_ready=1;
	return res;
}

int EUSART1_Write(console_type con, char c)
{
	char w_c=c;
	int rc;
	if(c=='\n' || c=='\r')
	{
		if(c=='\n' && z[con].w_did)
		{
			z[con].w_did=0;
			return 1;
		}
		w_c='\r';
		rc=c_write(con, &w_c);
		w_c='\n';
		z[con].w_did=1;
	} else z[con].w_did=0;
	rc=c_write(con, &w_c);
	return rc;
}

int EUSART1_is_tx_done(console_type con)
{
	if(z[con].w_ready && ((con==BT_CONSOLE && console_fd!=-1) || con==SERIAL_CONSOLE))
	{
		return 1;
	}
	else
	{
//		if(con==BT_CONSOLE)send_chars(SERIAL_CONSOLE,".");
		return 0;
	}
}


void send_chars(console_type con, char* x) {
    uint8_t i=0;
    while(x[i]!=0) EUSART1_Write(con, x[i++]);
    while (!EUSART1_is_tx_done(con)) if(stop_console[con]) return;
}


void empty_RXbuffer(console_type con) {
    while (EUSART1_is_rx_ready(con)) EUSART1_Read(con);
}

static int volatile timer4_expired[2];

static void Timer4Callback0( TimerHandle_t pxTimer )
{
	timer4_expired[0]=1;
}

static void Timer4Callback1( TimerHandle_t pxTimer )
{
	timer4_expired[1]=1;
}


static int getcert(console_type con, char* cert)
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
		send_chars(con, "Sent certificate or key, at the end please press Ctrl-Z...\n");
		timer4_expired[con]=0;
		TimerHandle_t Timer4= con==1 ? xTimerCreate("Timer41",300000/portTICK_RATE_MS,pdFALSE,NULL,Timer4Callback1) : xTimerCreate("Timer40",300000/portTICK_RATE_MS,pdFALSE,NULL,Timer4Callback0);
		xTimerStart(Timer4,0);
		z[con].gError=0;
		z[con].c_crlf=0;
		do
		{
			while(!timer4_expired[con] && !EUSART1_is_rx_ready(con)) vTaskDelay(1/portTICK_RATE_MS);
			c=EUSART1_Read(con);
			if(c=='\xff') continue;
			if(c==0x1a) break;
//			if(c=='\n') EUSART1_Write('!');
//			if(c=='\r') EUSART1_Write('?');
			EUSART1_Write(con,c);
			data[len++]=c;
		} while(!timer4_expired[con]);
		z[con].c_crlf=1;
		if(timer4_expired[con])
		{
			free(data);
			return 2;
		}
		if(mbedtls_md5_ret((const unsigned char*)data,len,md5))
		{
			send_chars(con, "Error while calculating MD5\n");
			free(data);
			return 2;
		}
		sprintf(str_md5,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",md5[0],md5[1],md5[2],md5[3],md5[4],md5[5],md5[6],md5[7],md5[8],md5[9],md5[10],md5[11],md5[12],md5[13],md5[14],md5[15]);
		sprintf(str,"\nCheck MD5 sum: %s\n",str_md5);
		send_chars(con, str);
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



void print_par(console_type con, char* p)
{
	char value[64];
	char y[256];
	_param* __params=_params;
    while(__params->type)
    {
        if(!strcmp(__params->c,p))
        {
        	get_value_from_nvs(p, z[con].hex, y, value);
        	send_chars(con, y);
            return;
        }
        __params++;
    }
}


void EUSART1_list(console_type con)
{
	char value[64];
	char y[256];
	int rc;
	list_init();
	do
	{
		rc=list(z[con].hex,y,value);
		send_chars(con, y);
	} while(rc==1);
}


uint8_t proceed(console_type con) {
    uint8_t i = 0, cmd, j;
    char par[16];
    //    printf("proceed %s\r\n",c_buf);
    z[con].c_buf[z[con].c_len] = 0;
    cmd = z[con].c_buf[i++];
    if(cmd==0) return 1;
    if(z[con].c_buf[1]=='X')
    {
        z[con].hex=1;
        i++;
    }
    else z[con].hex=0;
    if (cmd == 'Q' && z[con].c_buf[i] == 0) {
    	stop_console[SERIAL_CONSOLE]=1;
    	stop_console[BT_CONSOLE]=1;
        send_exit();
        return 0;
    }
    if (cmd == 'L' && z[con].c_buf[i] == 0) {
//        print_pars();
    	EUSART1_list(con);
        return 1;
    }
    while (z[con].c_buf[i] == ' ' || z[con].c_buf[i] == '\t') i++;
    j=0;
    while (z[con].c_buf[i] != ' ' && z[con].c_buf[i]!='=' && z[con].c_buf[i] != 0)
    {
    	par[j++] = z[con].c_buf[i++];
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
        print_par(con, par);
        return 1;
    }
    if(cmd=='G')
    {
    	return getcert(con, par);
    }
    if(cmd!='S') return 2;
//    i++;
    while (z[con].c_buf[i] == ' ' || z[con].c_buf[i] == '\t') i++;
    if (z[con].c_buf[i++] != '=') return 2;
    while (z[con].c_buf[i] == ' ' || z[con].c_buf[i] == '\t') i++;
    ip = 0;
    do {
    	z[con].val_buf[ip++] = z[con].c_buf[i];
    } while (z[con].c_buf[i++]);
    z[con].val_buf[ip]=0;
    set_value_in_nvs(par, z[con].val_buf);
    print_par(con, par);
    return 1;
}

static int volatile s2lp_console_timer_expired[2];

static void vTimerCallback0( TimerHandle_t pxTimer )
{
	s2lp_console_timer_expired[0]=1;
}

static void vTimerCallback1( TimerHandle_t pxTimer )
{
	s2lp_console_timer_expired[1]=1;
}


void start_x_shell(console_type con) {
    char c;
    uint8_t start = 0;
    int32_t x=con;
    char str[25];
    EUSART1_init(con);
    add_uid();
	send_chars(con, "\n Press any key to start console...\n");
	TimerHandle_t Timer3= con==SERIAL_CONSOLE ? xTimerCreate("Timer30",60000/portTICK_RATE_MS,pdFALSE,(void*)x,vTimerCallback0) : xTimerCreate("Timer31",60000/portTICK_RATE_MS,pdFALSE,(void*)x,vTimerCallback1);
    xTimerStart(Timer3,0);
    s2lp_console_timer_expired[con]=0;
    send_chars(con, ver);
    send_prompt();
    while (1)
    {
        if ((!start && s2lp_console_timer_expired[con]) || stop_console[con] )
        {
        	send_exit();
        	stop_console[con]=1;
//        	vTaskDelete(rtask[con]);
            return;
        }
        if (EUSART1_is_rx_ready(con))
        {
            c = EUSART1_Read(con);
            EUSART1_Write(con, c);
            if (c == 0x08) {
                EUSART1_Write(con, ' ');
                EUSART1_Write(con, c);
                z[con].c_len--;
                while (!EUSART1_is_tx_done(con));
                continue;
            }
            while (!EUSART1_is_tx_done(con) || stop_console[con]);
            start = 1;
            switch (c) {
                case '\r':
                    z[con].c_buf[z[con].c_len] = 0;
                    empty_RXbuffer(con);
                    uint8_t r = proceed(con);
                    if (r == 0)
                    {
//                    	vTaskDelete(rtask[con]);
                    	vTaskDelay(500 / portTICK_PERIOD_MS);
                    	return;
                    }
                    if (r != 1) send_error()
                    else send_prompt();
                    break;
                default:
                    if (c >= 0x61 && c <= 0x7A) c -= 0x20;
                    z[con].c_buf[z[con].c_len++] = c;
                    continue;
            }
            empty_RXbuffer(con);
            z[con].c_len = 0;
        }
    }
}



