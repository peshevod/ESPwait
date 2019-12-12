#include "shell.h"
#include "S2LP_Config.h"
#include <string.h>
#include <stdio.h>
#include "driver/uart.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_system.h"

_par _pars[]={
    {PAR_UI32,'F',{ 433000000UL } },  // base frequency
    {PAR_UI8,'M',{ 0xA0 } }, // modulation MOD_2FSK
    {PAR_UI32,'R',{ 12500UL }}, // datarate
    {PAR_UI32,'W',{ 50000UL }}, // bandwidth
    {PAR_UI32,'D',{ 12500UL }}, // freq_deviation
    {PAR_UI32,'S',{ 50000UL }}, // channel space
    {PAR_I32,'P',{ 16L }}, // power
    {PAR_UI8,'T',{ 1 }}, // transmit/rec
    {PAR_UI8,'L',{ 1 }}, // use LDO/bypass LDO
    {PAR_I32,'C',{ 21 }}, // channel
    {PAR_UI32,'E',{ 64 }}, // preamble length
    {PAR_UI32,'N',{ 0x00000301 }}, // id
    {PAR_UI32,'I',{ 30 }}, // interval in seconds
    {PAR_UI8,'X',{ 3 }}, // repeater
#ifdef HWVer3
    {PAR_UI8,'Y',{ 0x05 }}, // JP4 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP4 1 - norm, 0 - alarm
    {PAR_UI8,'Z',{ 0x06 }}, // JP5 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP5 1 - norm, 0 - alarm
#endif
#ifdef HWVer4
    {PAR_UI8,'Y',{ 0x01 }}, // JP4 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP4 1 - norm, 0 - alarm
    {PAR_UI8,'Z',{ 0x02 }}, // JP5 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP5 1 - norm, 0 - alarm
#endif
    {PAR_UI8,'G',{ 0x060 }}, // CRC mode - 0x00 -NO CRC, 0x20 - 8 bit, 0x40 - 16 bit 0x8005, 0x60 - 16 bit 0x1021
    {0,'\x00',{0}}
}; 

char t[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

char NL[2]={'\r','\n'};

char c_buf[BUF_LEN], val_buf[BUF_LEN];
uint8_t c_len;
uint8_t hex=0;
char prompt[] = {"\r\n> "};
char err[] = {"\r\nError\r\n> "};
char ex[] = {"\r\nExit\r\n"};
char commands[] = {'S', 'L', 'D'};
char ver[]={"=== S2-LP shell v 1.1.4 ===\r\n"};

TimerHandle_t Timer3;
nvs_handle_t nvsh;

void send_chars(char* x) {
    uint8_t len=strlen(x);
    uart_write_bytes(UART_NUM_0,x,len);
    uart_wait_tx_done(UART_NUM_0,1000);
}

uint8_t stringToUInt32(char* str, uint32_t* val) //it is a function made to convert the string value to integer value.
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

uint8_t stringToUInt8(char* str, uint8_t* val) //it is a function made to convert the string value to integer value.
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

uint8_t stringToInt32(char* str, int32_t* val) //it is a function made to convert the string value to integer value.
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

void _print_par(_par* par)
{
    if(par->type==PAR_UI32)
    {
        if(hex) ui32tox(par->u.ui32par, val_buf);
        else ui32toa(par->u.ui32par, val_buf);
    }
    if(par->type==PAR_I32)
    {
        if(hex) i32tox(par->u.i32par, val_buf);
        else i32toa(par->u.i32par, val_buf);
    }
    if(par->type==PAR_UI8)
    {
        if(hex) ui8tox(par->u.ui8par, val_buf);
        else ui8toa(par->u.ui8par, val_buf);
    }
	uart_write_bytes(UART_NUM_0,&par->c,1);
	char x='=';
	uart_write_bytes(UART_NUM_0,&x,1);
    uint8_t i = 0;
    while (val_buf[i]) {
    	uart_write_bytes(UART_NUM_0,&val_buf[i++],1);
    }
	uart_write_bytes(UART_NUM_0,NL,2);
    uart_wait_tx_done(UART_NUM_0,1000);
}

void print_par(char p)
{
    _par* __pars=_pars;
    while(__pars->type)
    {
        if(__pars->c==p)
        {
             _print_par(__pars);
             return;
        }
        __pars++;
    }
}

void print_pars()
{
    _par* __pars=_pars;
    while(__pars->type)
    {
         _print_par(__pars);
        __pars++;
    }
}

uint8_t set_par(char par, char* val_buf)
{
    _par* __pars=_pars;
    uint32_t ui32_v;
    int32_t i32_v;
    uint8_t ui8_v;
    char key[]={"s2lp.x"};
    key[5]=par;
    while(__pars->type)
    {
        if(__pars->c==par)
        {
            if(__pars->type==PAR_UI32)
            {
            	stringToUInt32(val_buf, &ui32_v);
            	if (nvs_set_u32(nvsh, key, ui32_v)!=ESP_OK) return 1;
            };
            if(__pars->type==PAR_I32)
            {
            	stringToInt32(val_buf, &i32_v);
            	if (nvs_set_i32(nvsh, key, i32_v)!=ESP_OK) return 1;
            };
            if(__pars->type==PAR_UI8)
            {
            	stringToUInt8(val_buf, &ui8_v);
            	if (nvs_set_u8(nvsh, key, ui8_v)!=ESP_OK) return 1;
            };
            return 0;
        };
        __pars++;
    }
    return 1;
}

void set_uid(uint32_t uid)
{
    _par* __pars=_pars;
    while(__pars->type)
    {
        if(__pars->c=='N')
        {
            __pars->u.ui32par=uid;
            return;
        }
        __pars++;
    }
}

uint8_t set_s(char p,void* s)
{
    char key[]={"s2lp.x"};
    key[5]=p;
    _par* __pars=_pars;
    while(__pars->type)
    {
        if(__pars->c==p)
        {
            if(__pars->type==PAR_UI32) if(nvs_get_u32(nvsh, key, (uint32_t*)s)!=ESP_OK) return 1;
            if(__pars->type==PAR_I32)  if(nvs_get_i32(nvsh, key, (int32_t*)s)!=ESP_OK) return 1;
            if(__pars->type==PAR_UI8)  if(nvs_get_u8(nvsh, key, (uint8_t*)s)!=ESP_OK) return 1;
            return 0;
        };
        __pars++;
    }
    return 1;
}

void get_uid(uint32_t* uid)
{
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
	memcpy(uid,&(mac[2]),4);
}

char* i32toa(int32_t i, char* b) {
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

char* ui32toa(uint32_t i, char* b) {
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

char* ui8toa(uint8_t i, char* b) {
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

char* ui8tox(uint8_t i, char* b)
{
    char* p = b;
    *p++='0';
    *p++='x';
    *p++=t[i>>4];
    *p++=t[i&0x0f];
    *p=0;
    return b;
}

char* i32tox(int32_t i, char* b)
{
    return ui32tox((uint32_t)i,b);
}


char* ui32tox(uint32_t i, char* b)
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


uint8_t proceed() {
    uint8_t i = 0, par, cmd;
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
        print_pars();
        return 1;
    }
    while (c_buf[i] == ' ' || c_buf[i] == '\t') i++;
    par = c_buf[i];
    uint8_t ip = 0, ip0 = 0xff;
    do {
        if (_pars[ip].c == par) {
            ip0 = ip;
            break;
        }
    } while (_pars[++ip].type);
    if (ip0 == 0xff) return 2;
    if (cmd == 'D') {
        if (c_buf[i + 1] == 0) {
            print_par(par);
            return 1;
        } else return 2;
    }
    i++;
    while (c_buf[i] == ' ' || c_buf[i] == '\t') i++;
    if (c_buf[i++] != '=') return 2;
    while (c_buf[i] == ' ' || c_buf[i] == '\t') i++;
    ip = 0;
    do {
        val_buf[ip++] = c_buf[i];
    } while (c_buf[i++]);
    if (set_par(par, val_buf)) return 2;
    print_par(par);
    return 1;
}

void start_x_shell(void) {
    char c, cmd;
    size_t len;
    uint8_t start = 0;
    uint32_t uid;
    //    printf("Start shell\r");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_open("s2lp", NVS_READWRITE, &nvsh));
    get_uid(&uid);
    set_uid(uid);
    c_len = 0;
    Timer3=xTimerCreate("Timer3",11000/portTICK_RATE_MS,pdFALSE,NULL,NULL);
    xTimerStart(Timer3,0);
//    SetTimer3(11000);
    send_chars(ver);
    send_prompt();
    while (1) {
        if (!start) {
//            if (TestTimer3()) {
        	if(!xTimerIsTimerActive(Timer3)) {
        	send_exit();
                return;
            }
        };
        uart_get_buffered_data_len(UART_NUM_0,&len);
        if (len!=0) {
            uart_read_bytes(UART_NUM_0,(uint8_t*)&c,1,1000);
            uart_write_bytes(UART_NUM_0,&c,1);
            if (c == 0x08) {
            	char x=' ';
            	uart_write_bytes(UART_NUM_0,&x,1);
            	uart_write_bytes(UART_NUM_0,&c,1);
                c_len--;
                uart_wait_tx_done(UART_NUM_0,1000);
                continue;
            }
            uart_wait_tx_done(UART_NUM_0,1000);
            start = 1;
            switch (c) {
                case '\r':
                case '\n':
                    c_buf[c_len] = 0;
                    uart_flush_input(UART_NUM_0);
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
            uart_flush_input(UART_NUM_0);
            c_len = 0;
        }
    }
}
