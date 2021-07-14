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
extern char* pCert;
extern char* pRoot;
extern char* pKey;




void EUSART1_init(console_type con)
{
	char tname[10];
	z[con].rb=xRingbufferCreate(RING_BUF_LEN,RINGBUF_TYPE_BYTEBUF);
    z[con].xSemaphore = xSemaphoreCreateMutex();
    z[con].gError=0;
	z[con].c_len=0;
	z[con].hex=0;
	z[con].w_ready=1;
	z[con].lenr=0;
	z[con].bufr_len=0;
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
			} else xSemaphoreGive(z[con].xSemaphore);
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}



static char EUSART1_Read(console_type con)
{
	size_t size;
	char c;
	c=-1;
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
	return c;
}

static int EUSART1_is_rx_ready(console_type con)
{
	UBaseType_t uxFree;
	UBaseType_t uxRead;
	UBaseType_t uxWrite;
	UBaseType_t uxAcquire;
	UBaseType_t uxItemsWaiting;
	if(z[con].bufr_len!=0) return 1;
	xSemaphoreTake(z[con].xSemaphore,portMAX_DELAY);
	vRingbufferGetInfo(z[con].rb, &uxFree, &uxRead, &uxWrite, &uxAcquire, &uxItemsWaiting);
	xSemaphoreGive(z[con].xSemaphore);
	if(uxItemsWaiting>0) return 1;
	else return 0;
}

static int EUSART1_Write(console_type con, char c)
{
	int k=12000, res;
	z[con].w_ready=0;
	res=-1;
	do
	{
		if((con==BT_CONSOLE && console_fd!=-1) || con==SERIAL_CONSOLE)
		{
			if(con==BT_CONSOLE) res=write(console_fd,&c,1);
			if(con==SERIAL_CONSOLE) res=uart_write_bytes(UART_NUM_0,&c,1);
			if(res==1)
			{
				z[con].w_ready=1;
				return 1;
			}
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	} while (k-->0);
	return res;
}


int send_chars(console_type con, char* x) {
    uint8_t i=0;
    while(x[i]!=0)
	{
    	if(EUSART1_Write(con, x[i++])==1) continue;
    	return -1;
	}
    return 0;
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
	char str_md5[40];
	char str[64];
	if(!strcmp(cert,"KEY") || !strcmp(cert,"ROOT") || !strcmp(cert,"CERT") )
	{
		char* data=(char*)malloc(4096);
		send_chars(con, "Please wait 1s, then sent certificate or key, at the end press Ctrl-Z...\n");
		timer4_expired[con]=0;
		TimerHandle_t Timer4= con==1 ? xTimerCreate("Timer41",300000/portTICK_RATE_MS,pdFALSE,NULL,Timer4Callback1) : xTimerCreate("Timer40",300000/portTICK_RATE_MS,pdFALSE,NULL,Timer4Callback0);
		xTimerStart(Timer4,0);
		z[con].gError=0;
		empty_RXbuffer(con);
		do
		{
			while(!timer4_expired[con] && !EUSART1_is_rx_ready(con)) vTaskDelay(1/portTICK_RATE_MS);
			c=EUSART1_Read(con);
			if(c=='\xff') continue;
			if( c==0x1a || c==0x00 )
			{
				data[len++]=0;
				break;
			}
//			if(c=='\n') EUSART1_Write('!');
//			if(c=='\r') EUSART1_Write('?');
			EUSART1_Write(con,c);
			data[len++]=c;
		} while(!timer4_expired[con]);
		if(timer4_expired[con])
		{
			free(data);
			return 2;
		}
		switch(write_cert_to_nvs(cert,data,len,str_md5))
		{
			case -1:
				send_chars(con, "Error while calculating MD5\n");
				free(data);
				return 2;
			case -2:
				send_chars(con, "Error while writing MD5 to flash\n");
				free(data);
				return 2;
			case -3:
				send_chars(con, "Wrong cert name\n");
				free(data);
				return 2;
			case 0:
				sprintf(str,"\nCheck MD5 sum: %s\n",str_md5);
				send_chars(con, str);
				free(data);
				return 1;
		}
		free(data);
	}
	return 2;
}

/* Compare upper(par) with c
 * if shrt==1 compare only length(c) chars, if shrt==0 compare strongly full names
 * return 1 if ok, 0 - not OK
 */

static uint8_t parcmp(char* par,char *c, uint8_t shrt)
{
    char s;
    uint8_t j=0;
    while((s=par[j]))
    {
        if(c[j]==0) return shrt;
        if( s >= 0x61 && s <= 0x7A ) s-=0x20;
        if(s!=c[j]) return 0;
        j++;
    }
    if(c[j]) return 0;
    return 1;
}

uint8_t set_s(char* p,void* s)
{
    _par_t* __pars=_pars;
    while(__pars->type)
    {
        if(parcmp(__pars->c,p,0))
        {
            if(__pars->type==PAR_UI32) *((uint32_t*)s)=__pars->u.ui32par;
            if(__pars->type==PAR_I32)  *((int32_t*)s)=__pars->u.i32par;
            if(__pars->type==PAR_UI8)  *((uint8_t*)s)=__pars->u.ui8par;
            if(__pars->type==PAR_EUI64) for(uint8_t j=0;j<8;j++) ((uint8_t*)s)[j]=__pars->u.eui[j];
            if(__pars->type==PAR_KEY128) for(uint8_t j=0;j<16;j++) ((uint8_t*)s)[j]=__pars->u.key[j];
            return 0;
        };
        __pars++;
    }
    return 1;
}

uint8_t set_par(char* par, char* val_buf)
{
    _par_t* __pars=_pars;
    int i=1;
    while(__pars->type)
    {
        if(parcmp(__pars->c,par,0))
        {
            if(!strcmp(__pars->c,"Erase_EEPROM"))
            {
//                erase_EEPROM_Data();
                return 0;
            }
            if(__pars->type==PAR_UI32)
            {
                if (stringToUInt32(val_buf, &(__pars->u.ui32par))) return 1;
                for(uint8_t j=0;j<4;j++) DATAEE_WriteByte(i+3-j,((uint8_t*)(&(__pars->u.ui32par)))[j]);
            }
            else if(__pars->type==PAR_I32)
            {
                if (stringToInt32(val_buf, &(__pars->u.i32par))) return 1;
                for(uint8_t j=0;j<4;j++) DATAEE_WriteByte(i+3-j,((uint8_t*)(&(__pars->u.i32par)))[j]);
            }
            else if(__pars->type==PAR_UI8)
            {
                if (stringToUInt8(val_buf, &(__pars->u.ui8par))) return 1;
                DATAEE_WriteByte(i,__pars->u.ui8par);
            }
            else if(__pars->type==PAR_KEY128)
            {
                if(strlen(val_buf)!=32) return 1;
                d[0]='0';
                d[1]='x';
                d[4]=0;
                if(!strcmp(__pars->c,"AppKey"))
                {
                    if(!show_hidden)
                    {
                        uint8_t cmp;
                        for(uint8_t j=0;j<16;j++)
                        {
                            d[2]=val_buf[2*j];
                            d[3]=val_buf[2*j+1];
                            if(stringToUInt8(d,&cmp)) return 1;
                            if(cmp!=__pars->u.key[j]) return 1;
                        }
                        show_hidden=VISIBLE;
                        return 0;
                    }
                }
                for(uint8_t j=0;j<16;j++)
                {
                    d[2]=val_buf[2*j];
                    d[3]=val_buf[2*j+1];
                    if (stringToUInt8(d, &(__pars->u.key[j]))) return 1;
                    DATAEE_WriteByte(i+j,__pars->u.key[j]);
                }
            }
            else if(__pars->type==PAR_EUI64)
            {
                if(strlen(val_buf)!=16) return 1;
                d[0]='0';
                d[1]='x';
                d[4]=0;
                for(uint8_t j=0;j<8;j++)
                {
                    d[2]=val_buf[2*j];
                    d[3]=val_buf[2*j+1];
                    if (stringToUInt8(d, &(__pars->u.eui[j]))) return 1;
                    DATAEE_WriteByte(i+j,__pars->u.eui[j]);
                }
            }
            return 0;
        }
        if(__pars->type==PAR_UI8) i++;
        else if(__pars->type==PAR_UI32 || __pars->type==PAR_I32 ) i+=4;
        else if(__pars->type==PAR_KEY128) i+=16;
        else if(__pars->type==PAR_EUI64) i+=8;
        __pars++;
    }
    return 1;
}



static int print_par(console_type con, char* p)
{
	char value[64];
	char y[256];
	_param* __params=_params;
    while(__params->type)
    {
        if(!strcmp(__params->c,p))
        {
        	get_value_from_nvs(p, z[con].hex, y, value);
        	return send_chars(con, y);
        }
        __params++;
    }
    return -1;
}


static int EUSART1_list(console_type con)
{
	char value[64];
	char y[256];
	int rc;
	list_init();
	do
	{
		rc=list(z[con].hex,y,value);
		if(send_chars(con, y)!=0) return -1;
	} while(rc==1);
	return 0;
}

static int EUSART1_help(console_type con)
{
	char* lines[]={
			"H - print this help\n",
			"L(x) - print all parameters\n",
			"D(x) <p> - print parameter <p>\n",
			"S <p>=<v> - set parameter <p> to <v>",
			"G KEY - upload private key\n",
			"G CERT - upload thing certificate\n",
			"G ROOT - upload root certificate\n"
	};
	for(uint8_t i=0;i<7;i++) if(send_chars(con,lines[i])!=0) return -1;
	return 0;

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
    	if(EUSART1_list(con)!=0) return -1;
        return 1;
    }
    if (cmd == 'H' && z[con].c_buf[i] == 0) {
    	if(EUSART1_help(con)!=0) return -1;
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
        if(print_par(con, par)!=0) return -1;
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
    if(print_par(con, par)!=0) return -1;
    return 1;
}

static int volatile s2lp_console_timer_expired[2];

static void vTimerCallback0( TimerHandle_t pxTimer )
{
	s2lp_console_timer_expired[0]=1;
	ESP_LOGI("vTimerCallback0", "Timer expired");
}

static void vTimerCallback1( TimerHandle_t pxTimer )
{
	s2lp_console_timer_expired[1]=1;
	ESP_LOGI("vTimerCallback1", "Timer expired");
}


int start_x_shell(console_type con) {
    char c;
    uint8_t start = 0;
    int32_t x=con;
    int rc;
    EUSART1_init(con);
    add_uid();
	if(send_chars(con, "\n Press any key to start console...\n")!=0)
	{
        stop_console[con]=1;
    	return -1;
	}
	TimerHandle_t Timer3= con==SERIAL_CONSOLE ? xTimerCreate("Timer30",60000/portTICK_RATE_MS,pdFALSE,(void*)x,vTimerCallback0) : xTimerCreate("Timer31",60000/portTICK_RATE_MS,pdFALSE,(void*)x,vTimerCallback1);
    if(Timer3==NULL)
    {
    	ESP_LOGI("start_x_shell","Timer not created, console=%d",con);
        stop_console[con]=1;
    	return -1;
    }
	if(xTimerStart(Timer3,0)!=pdPASS)
	{
    	ESP_LOGI("start_x_shell","Timer not started, console=%d",con);
        stop_console[con]=1;
    	return -1;
	}
    s2lp_console_timer_expired[con]=0;
    if(send_chars(con, ver)!=0)
    {
    	stop_console[con]=1;
    	return -1;
    }
    send_prompt();
    while (1)
    {
        if (stop_console[con])
        {
        	send_exit();
        	return 0;
        }
    	if ((!start && s2lp_console_timer_expired[con]))
        {
        	send_exit();
        	stop_console[con]=1;
            return 0;
        }
        if ((rc=EUSART1_is_rx_ready(con)))
        {
            if(rc==-1)
            {
            	stop_console[con]=1;
            	return -1;
            }
        	c = EUSART1_Read(con);
            if(EUSART1_Write(con, c)!=1)
			{
				stop_console[con]=1;
				return -1;
			}
            if (c == 0x08) {
                if(EUSART1_Write(con, ' ')!=1 || EUSART1_Write(con, c)!=1)
				{
                	stop_console[con]=1;
                    return -1;
				}
                z[con].c_len--;
                continue;
            }
            start = 1;
            if(c=='\r' || c== '\n')
            {
				z[con].c_buf[z[con].c_len] = 0;
				uint8_t r = proceed(con);
				if (r == 0)
				{
					vTaskDelay(500 / portTICK_PERIOD_MS);
					stop_console[con]=1;
					return 0;
				}
				if (r != 1) send_error()
				else send_prompt();
				empty_RXbuffer(con);
				z[con].c_len = 0;
            }
            else
            {
				if (c >= 0x61 && c <= 0x7A) c -= 0x20;
				z[con].c_buf[z[con].c_len++] = c;
            }
        } else vTaskDelay(100/portTICK_RATE_MS);
    }
}



