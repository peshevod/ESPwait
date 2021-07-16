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

#include "sys/unistd.h"
#include "shell.h"

char t[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

char prompt[] = {"ESPWait> "};
char err[] = {"Error\nESPWait> "};
char ex[] = {"Exit\n"};
char commands[] = {'S', 'L', 'D'};
char ver[]={"=== S2-LP shell v 1.1.5 ===\n"};
char b[BUF_LEN], val_buf[BUF_LEN];
uint8_t show_hidden=0;

static exchange_par_t z[2];
extern _par_t _pars[];
extern int volatile console_fd;
uint8_t stop_console[2];
TaskHandle_t rtask[2];
uint8_t hex=0;
char d[5];

static uint8_t stringToUInt32(char* str, uint32_t* val) //it is a function made to convert the string value to integer value.
{
    uint8_t i = 0;
    uint32_t sum = 0;
    if(str[0]=='0' && (str[1]=='x' || str[1]=='X'))
    {
        i+=2;
        while(str[i] != 0)
        {
           if (str[i] >= 0x30 && str[i] <= 0x39) sum=sum*16+(str[i]-0x30);
           else if(str[i] >= 0x41 && str[i] <= 0x46) sum=sum*16+(str[i]-0x41+10);
           else if(str[i] >= 0x61 && str[i] <= 0x66) sum=sum*16+(str[i]-0x41+10);
           else return 1;
           i++;
        }
    }
    else
    {
        while (str[i] != '\0') //string not equals to null
        {

            if (str[i] < 48 || str[i] > 57) return 1; // ascii value of numbers are between 48 and 57.
            else {
                sum = sum * 10 + (str[i] - 48);
                i++;
            }
        }
    }
    *val = sum;
    return 0;
}

static uint8_t stringToUInt8(char* str, uint8_t* val) //it is a function made to convert the string value to integer value.
{
    int8_t i = -1;
    uint8_t sum = 0;
    if(str[0]=='0' && (str[1]=='x' || str[1]=='X'))
    {
        i+=2;
        while(str[++i] != 0)
        {
           if(i>=4) return 1;
           if (str[i] >= 0x30 && str[i] <= 0x39) sum=sum*16+(str[i]-0x30);
           else if(str[i] >= 0x41 && str[i] <= 0x46) sum=sum*16+(str[i]-0x41+10);
           else if(str[i] >= 0x61 && str[i] <= 0x66) sum=sum*16+(str[i]-0x41+10);
           else return 1;
        }
    }
    else
    {
        while (str[++i] != 0);
        if (i > 3) return 1;
        if (i == 3) {
            if (str[0] > 0x32) return 1;
            if (str[0] == 0x32) {
                if (str[1] > 0x35) return 1;
                if (str[0]==0x32 && str[1] == 0x35 && str[2] > 0x35) return 1;
            }
        }
        i = 0;
        while (str[i] != '\0') //string not equals to null
        {
            if (str[i] < 48 || str[i] > 57) return 1; // ascii value of numbers are between 48 and 57.
            else {
                sum = sum * 10 + (str[i] - 48);
                i++;
            }
        }
    }
    *val = sum;
    return 0;
}

static uint8_t stringToInt32(char* str, int32_t* val) //it is a function made to convert the string value to integer value.
{
    uint8_t i = 0, sign = 0;
    int32_t sum = 0;
    if (str[0] == '-') {
        sign = 1;
        i = 1;
    }
    if(str[i]=='0' && (str[i+1]=='x' || str[i+1]=='X'))
    {
        i+=2;
        while(str[i] != 0)
        {
           if((i-sign)>=10) return 1;
           if (str[i] >= 0x30 && str[i] <= 0x39) sum=sum*16+(str[i]-0x30);
           else if(str[i] >= 0x41 && str[i] <= 0x46) sum=sum*16+(str[i]-0x41+10);
           else if(str[i] >= 0x61 && str[i] <= 0x66) sum=sum*16+(str[i]-0x41+10);
           else return 1;
           i++;
        }
    }
    else
    {
        while (str[i] != '\0') //string not equals to null
        {

            if (str[i] < 48 || str[i] > 57) return 1; // ascii value of numbers are between 48 and 57.
            else {
                sum = sum * 10 + (str[i] - 48);
                i++;
            }
        }
    }
    if (sign) *val = -sum;
    else *val = sum;
    return 0;
}

static char* i32toa(int32_t i, char* b) {
    char const digit[] = "0123456789";
    char* p = b;
    if (i < 0) {
        *p++ = '-';
        i *= -1;
    }
    int32_t shifter = i;
    do { //Move to where representation ends
        ++p;
        shifter = shifter / 10;
    } while (shifter);
    *p = '\0';
    do { //Move back, inserting digits as u go
        *--p = digit[i % 10];
        i = i / 10;
    } while (i);
    return b;
}

static char* ui32toa(uint32_t i, char* b) {
    char const digit[] = "0123456789";
    char* p = b;
    uint32_t shifter = i;
    do { //Move to where representation ends
        ++p;
        shifter = shifter / 10;
    } while (shifter);
    *p = '\0';
    do { //Move back, inserting digits as u go
        *--p = digit[i % 10];
        i = i / 10;
    } while (i);
    return b;
}

static char* ui8toa(uint8_t i, char* b) {
    char const digit[] = "0123456789";
    char* p = b;
    uint8_t shifter = i;
    do { //Move to where representation ends
        ++p;
        shifter = shifter / 10;
    } while (shifter);
    *p = '\0';
    do { //Move back, inserting digits as u go
        *--p = digit[i % 10];
        i = i / 10;
    } while (i);
    return b;
}

static char* ui8tox(uint8_t i, char* b)
{
    char* p = b;
    *p++='0';
    *p++='x';
    *p++=t[i>>4];
    *p++=t[i&0x0f];
    *p=0;
    return b;
}


static char* ui32tox(uint32_t i, char* b)
{
    uint8_t* ch;
    ch=((uint8_t*)(&i));
    char* p = b;
    *p++='0';
    *p++='x';
    *p++=t[ch[3]>>4];
    *p++=t[ch[3]&0x0F];
    *p++=t[ch[2]>>4];
    *p++=t[ch[2]&0x0F];
    *p++=t[ch[1]>>4];
    *p++=t[ch[1]&0x0F];
    *p++=t[ch[0]>>4];
    *p++=t[ch[0]&0x0F];
    *p=0;
    return b;
}

static char* i32tox(int32_t i, char* b)
{
    return ui32tox((uint32_t)i,b);
}

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


void empty_RXbuffer(console_type con) {
    while (EUSART1_is_rx_ready(con)) EUSART1_Read(con);
}

bool EUSART1_is_tx_done(console_type con)
{
	return (z[con].w_ready==1);
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
            if(__pars->type==PAR_STR) strcpy((char*)s,__pars->u.str);
            return 0;
        };
        __pars++;
    }
    return 1;
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

void printVar(console_type con, char* text, par_type_t type, void* var, bool hex, bool endline)
{
    uint8_t j0=8;
    send_chars(con,text);
    switch(type)
    {
        case PAR_UI32:
            if(hex) send_chars(con,ui32tox(*((uint32_t*)var),b)); else send_chars(con,ui32toa(*((uint32_t*)var),b));
            break;
        case PAR_I32:
            if(hex) send_chars(con,i32tox(*((int32_t*)var),b)); else send_chars(con,i32toa(*((int32_t*)var),b));
            break;
        case PAR_UI8:
            if(hex) send_chars(con,ui8tox(*((uint8_t*)var),b)); else send_chars(con,ui8toa(*((uint8_t*)var),b));
            break;
        case PAR_KEY128:
            j0=16;
        case PAR_EUI64:
            for(uint8_t j=0;j<j0;j++)
            {
                send_chars(con," ");
                if(hex) send_chars(con,ui8tox(((uint8_t*)var)[j],b)); else send_chars(con,ui8toa(((uint8_t*)var)[j],b));
            }
            break;
        case PAR_STR:
            send_chars(con,(char*)var);
    }
    if(endline) send_chars(con,"\r\n");
}


static void _print_par(console_type con, _par_t* par)
{
    if(par->visible==HIDDEN && !show_hidden) return;
    if(par->type==PAR_UI32)
    {
        if(hex) ui32tox(par->u.ui32par, val_buf);
        else ui32toa(par->u.ui32par, val_buf);
    }
    else if(par->type==PAR_I32)
    {
        if(hex) i32tox(par->u.i32par, val_buf);
        else i32toa(par->u.i32par, val_buf);
    }
    else if(par->type==PAR_UI8)
    {
        if(hex) ui8tox(par->u.ui8par, val_buf);
        else ui8toa(par->u.ui8par, val_buf);
    }
    else if(par->type==PAR_KEY128)
    {
        for(uint8_t j=0;j<16;j++)
        {
            ui8tox(par->u.key[j], d);
            val_buf[2*j]=d[2];
            val_buf[2*j+1]=d[3];
        }
        val_buf[32]=0;
    }
    else if(par->type==PAR_EUI64)
    {
        for(uint8_t j=0;j<8;j++)
        {
            ui8tox(par->u.eui[j], d);
            val_buf[2*j]=d[2];
            val_buf[2*j+1]=d[3];
        }
        val_buf[16]=0;
    }
    else if(par->type==PAR_STR)
    {
        strcpy(val_buf,par->u.str);
    }
    else return;
    char* s=par->c;
    while(*s!=0)
    {
        EUSART1_Write(con,*s);
        s++;
    }
    EUSART1_Write(con, '=');
    uint8_t i = 0;
    while (val_buf[i]) {
        EUSART1_Write(con, val_buf[i++]);
        while (!EUSART1_is_tx_done(con));
    }
    EUSART1_Write(con,' ');
    i=0;
    while (par->d[i]) {
        EUSART1_Write(con, par->d[i++]);
        while (!EUSART1_is_tx_done(con));
    }
    EUSART1_Write(con,'\r');
    EUSART1_Write(con,'\n');
    while (!EUSART1_is_tx_done(con));
}

static void print_par(console_type con, char* p)
{
    _par_t* __pars=_pars;
    while(__pars->type)
    {
        if(parcmp(__pars->c,p,0))
        {
             _print_par(con, __pars);
             return;
        }
        __pars++;
    }
}

static void print_pars(console_type con)
{
    _par_t* __pars=_pars;
    while(__pars->type)
    {
         _print_par(con,__pars);
        __pars++;
    }
}


uint8_t set_par(console_type con, char* par, char* val_buf)
{
    _par_t* __pars=_pars;
    esp_err_t err;
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
                if((err=Write_u32_params(__pars->c,__pars->u.ui32par))!=ESP_OK) return err;
            }
            else if(__pars->type==PAR_I32)
            {
                if (stringToInt32(val_buf, &(__pars->u.i32par))) return 1;
//                send_chars(con,"before write");
                if((err=Write_i32_params(__pars->c,__pars->u.i32par))!=ESP_OK)
				{
//                	printf("err=%s\n",esp_err_to_name(err));
                	return err;
				}
            }
            else if(__pars->type==PAR_UI8)
            {
                if (stringToUInt8(val_buf, &(__pars->u.ui8par))) return 1;
                if((err=Write_u8_params(__pars->c,__pars->u.ui8par))!=ESP_OK) return err;
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
                }
                if((err=Write_key_params(__pars->c,__pars->u.key))!=ESP_OK) return err;
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
                }
                if((err=Write_eui_params(__pars->c,__pars->u.eui))!=ESP_OK) return err;
            }
            else if(__pars->type==PAR_STR)
            {
                strcpy(__pars->u.str, val_buf);
            	if((err=Write_str_params(__pars->c,__pars->u.str))!=ESP_OK) return err;
            }
            else return 1;
//            send_chars(con,"before commit");
            if((err=Commit_params())!=ESP_OK) return err;
            return 0;
        }
        __pars++;
    }
    return 1;
}

static uint8_t proceed(console_type con) {
    uint8_t i = 0,cmd,j,s;
    char par[16];
    //    printf("proceed %s\r\n",c_buf);
    z[con].c_buf[z[con].c_len] = 0;
    cmd = z[con].c_buf[i++];
    if(cmd==0) return 1;
    if(z[con].c_buf[1]=='X')
    {
        hex=1;
        i++;
    }
    else hex=0;
    if (cmd == 'Q' && z[con].c_buf[i] == 0) {
    	stop_console[SERIAL_CONSOLE]=1;
    	stop_console[BT_CONSOLE]=1;
        send_exit();
        return 0;
    }
    if (cmd == 'L' && z[con].c_buf[i] == 0) {
        print_pars(con);
        return 1;
    }
    while (z[con].c_buf[i] == ' ' || z[con].c_buf[i] == '\t') i++;
    j=0;
    s=z[con].c_buf[i];
    while(s!=' ' && s!='\t' && s!=0 && s!='=')
    {
        par[j++] = s;
        s=z[con].c_buf[++i];
    }
    par[j]=0;
    uint8_t ip = 0, ip0 = 0xff;
    j=0;
    do {
        if (parcmp(_pars[ip].c,par,1)) {
            ip0 = ip;
            j++;
        }
    } while (_pars[++ip].type);
    if (j!=1) return 2;
    j=0;
    while((s=_pars[ip0].c[j]))
    {
        if( s >= 0x61 && s <= 0x7A ) s-=0x20;
        par[j++]=s;
    }
    par[j]=0;
    /*send_chars("\r\n par=");
    send_chars(par);
    send_chars("\r\n");*/
    if (cmd == 'D') {
        if (z[con].c_buf[i] == 0) {
            print_par(con,par);
            return 1;
        } else return 2;
    }
    while (z[con].c_buf[i] == ' ' || z[con].c_buf[i] == '\t') i++;
    if (z[con].c_buf[i++] != '=') return 2;
    while (z[con].c_buf[i] == ' ' || z[con].c_buf[i] == '\t') i++;
    ip = 0;
    do {
        val_buf[ip++] = z[con].c_buf[i];
    } while (z[con].c_buf[i++]);
    if (set_par(con, par, val_buf)) return 2;
    print_par(con, par);
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
    int rc;
    int32_t x=con;
    //    printf("Start shell\r");

    EUSART1_init(con);
    z[con].c_len = 0;
    show_hidden=0;
//    SetTimer3(11000);
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
				if (r != 1)
				{
					send_error()
				}
				else
				{
					send_prompt()
				}
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


