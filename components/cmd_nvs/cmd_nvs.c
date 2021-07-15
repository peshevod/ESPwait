/* Console example — NVS commands

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <inttypes.h>
#include "esp_console.h"
#include "esp_log.h"
//#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_nvs.h"

_par_t _pars[]={
	{PAR_UI32,"Frequency",{ 864100000UL },"Base frequency, Hz",VISIBLE},
	{PAR_UI8,"Channel",{ 2 },"Lora Channel number in channel list, 255 - no channel selected",VISIBLE},
	{PAR_UI8,"Modulation",{ 0 }, "Modulation 0: lora, 1-FSK no shaping, 2: FSK BT=1, 3: FSK BT=0.5, 4: FSK BT=0.3",VISIBLE },
	{PAR_UI32,"FSK_BitRate",{ 50000UL }, "FSK Datarate bit/s",VISIBLE },
	{PAR_UI8,"FSK_BW",{ 0b01011 }, "bits: <ab><cde> <ab>: 00,01,10 BW= FXOSC/( 16*( (4+<ab>) * 2^<cde> ) )  FSK Bandwidth, Hz 0b01011 = BW 50kHz",VISIBLE },
	{PAR_UI8,"FSK_AFC_BW",{ 0b10010 }, "bits: <ab><cde> <ab>: 00,01,10 BW= FXOSC/( 16*( (4+<ab>) * 2^<cde> ) )  FSK AFC Bandwidth, Hz 0b10010 = AFC BW 83,3kHz",VISIBLE },
	{PAR_UI32,"BW",{ 125000UL }, "LORA Bandwidth, Hz 125 or 250 or 500 kHz",VISIBLE },
	{PAR_UI32,"Deviation",{ 25000UL }, "FSK Frequency deviation, Hz",VISIBLE },
	{PAR_UI8,"SF",{ 7 }, "LORA Spreading Factor (bitrate) 7-12",VISIBLE },
	{PAR_UI8,"CRC",{ 1 }, "LORA 1: CRC ON, 0: CRC OFF",VISIBLE },
	{PAR_UI8,"FEC",{ 1 }, "LORA FEC 1: 4/5, 2: 4/6 3: 4/7 4: 4/8",VISIBLE },
	{PAR_UI8,"Header_Mode",{ 0 }, "LORA Header 0: Explicit, 1: Implicit",VISIBLE },
	{PAR_I32,"Power",{ 1 }, "Power, dbm",VISIBLE },
	{PAR_UI8,"Boost",{ 0 }, "PA Boost 1: PABoost ON 0: PABoost OFF",VISIBLE },
	{PAR_UI8,"IQ_Inverted",{ 0 }, "LORA 0: IqInverted OFF 1: IqInverted ON",VISIBLE },
	{PAR_I32,"SNR",{ -125 }, "FSK Packet SNR",VISIBLE },
	{PAR_UI8,"Mode",{ 1 }, "Mode 0:receive, 1:transmit, 2:device, 3:simple gateway",VISIBLE },
	{PAR_UI32,"Preamble_Len",{ 8 }, "Preamble length",VISIBLE },
	{PAR_UI32,"UID",{ 0x12345678 }, "UID",VISIBLE },
	{PAR_UI8,"LORA_SyncWord",{ 0x34 }, "LORA Sync Word",VISIBLE },
	{PAR_UI8,"FSK_SyncWordLen",{ 3 }, "FSK Sync Word length 1-3",VISIBLE },
	{PAR_UI32,"FSK_SyncWord",{ 0xC194C100 }, "FSK Sync Words ",VISIBLE },
	{PAR_UI32,"Interval",{ 30 }, "Interval between actions (trans or rec), sec.",VISIBLE },
	{PAR_UI8,"Rep",{ 3 }, "Number of repeated messages in trans mode",VISIBLE },
	{PAR_UI8,"Y",{ 0x01 }, "JP4 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP4 1 - norm, 0 - alarm",VISIBLE },
	{PAR_UI8,"Z",{ 0x02 }, "JP5 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP5 1 - norm, 0 - alarm",VISIBLE },
	{PAR_UI8,"SPI_Trace",{ 0 }, "Tracing SPI 0:OFF 1:ON",VISIBLE },
	{PAR_UI8,"JSNumber",{ 1 }, "Select Join Server - 1, 2 or 3",VISIBLE },
	{PAR_UI32,"NetID",{ 0x00000000 }, "Network Id",VISIBLE },
	{PAR_I32,"RX1_offset",{ -40 }, "Offset(ms) to send ack",VISIBLE },
	{PAR_KEY128,"AppKey",{.key={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f,0x10}}, "Application Key 128 bit",VISIBLE  },
	{PAR_EUI64,"Dev0Eui",{.eui={0,0,0,0,0,0,0,0}}, "Dev0Eui 64",VISIBLE },
	{PAR_EUI64,"Dev1Eui",{.eui={0x20,0x37,0x11,0x32,0x10,0x19,0x00,0x70}}, "Dev1Eui 64",VISIBLE  },
	{PAR_EUI64,"Dev2Eui",{.eui={0x20,0x37,0x11,0x32,0x11,0x15,0x00,0x80}}, "Dev2Eui 64",VISIBLE  },
	{PAR_EUI64,"Dev3Eui",{.eui={0x20,0x37,0x11,0x32,0x13,0x13,0x00,0x10}}, "Dev3Eui 64",VISIBLE  },
	{PAR_EUI64,"Dev4Eui",{.eui={0,0,0,0,0,0,0,0}}, "Dev4Eui 64",VISIBLE  },
	{PAR_EUI64,"Dev5Eui",{.eui={0,0,0,0,0,0,0,0}}, "Dev5Eui 64",VISIBLE  },
	{PAR_EUI64,"Dev6Eui",{.eui={0,0,0,0,0,0,0,0}}, "Dev6Eui 64",VISIBLE  },
	{PAR_EUI64,"Dev7Eui",{.eui={0,0,0,0,0,0,0,0}}, "Dev7Eui 64",VISIBLE  },
	{PAR_EUI64,"Join0Eui",{.eui={0,0,0,0,0,0,0,0}}, "Join0Eui 64",VISIBLE  },
	{PAR_EUI64,"Join1Eui",{.eui={0x20,0x37,0x11,0x32,0x10,0x19,0x00,0x70}}, "Join1Eui 64",VISIBLE  },
	{PAR_EUI64,"Join2Eui",{.eui={0x20,0x37,0x11,0x32,0x11,0x15,0x00,0x80}}, "Join2Eui 64",VISIBLE  },
	{PAR_EUI64,"Join3Eui",{.eui={0x20,0x37,0x11,0x32,0x13,0x13,0x00,0x10}}, "Join3Eui 64",VISIBLE  },
	{PAR_UI32,"EEPROM_types",{ 0xFFFFFFFF }, "Types of 32 Eui in EEPROM 0-DevEui, 1-JoinEui",VISIBLE },
	{PAR_UI8,"Erase_EEPROM",{0},"If set erase EEPROM",HIDDEN},
	{PAR_STR,"SSID",{.str="Paritet"},"SSID",VISIBLE},
	{PAR_STR,"PASSWD",{.str="narrowcartoon617"},"Password for ssid",VISIBLE},
	{0,NULL,{0},NULL,HIDDEN}
};

nvs_handle_t nvs;
uint8_t s2lp_console_ex=0;

static char eeprom_namespace[16] = "params";
static char eeprom_partition[16] = "params";
static const char *TAG = "cmd_nvs";


static void make_deveui(void);
static void get_uid(uint32_t* uid);
static esp_err_t add_uid(void);


esp_err_t Sync_EEPROM(void)
{
    _par_t* __pars=_pars;
    esp_err_t err;
    uint8_t v8;
    size_t len;
    void* p;

	err=nvs_flash_init_partition(eeprom_partition);
//	printf("nvs_flash_init_partition result=%s\n",esp_err_to_name(err));

	err = nvs_open_from_partition(eeprom_partition, eeprom_namespace, NVS_READWRITE, &nvs);
//	printf("nvs_open namespace result=%s\n",esp_err_to_name(err));
    if (err != ESP_OK) {
    	return err;
    }
    if ((err = nvs_get_u8(nvs, "eeprom", &v8)) != ESP_OK)
    {
    	return err;
    }

    if(v8)
    {
        while(__pars->type)
        {
            if(__pars->type==PAR_UI8)
            {
                if ((err = nvs_get_u8(nvs, __pars->c, &__pars->u.ui8par)) != ESP_OK)
           		{
                	return err;
           		}
            }
            else if(__pars->type==PAR_UI32)
            {
                if ((err = nvs_get_u32(nvs, __pars->c, &__pars->u.ui32par)) != ESP_OK)
           		{
                	return err;
           		}
            }
            else if(__pars->type==PAR_I32)
            {
                if ((err = nvs_get_i32(nvs, __pars->c, &__pars->u.i32par)) != ESP_OK)
           		{
                	return err;
           		}
            }
            else if(__pars->type==PAR_KEY128)
            {
            	if((err = nvs_get_blob(nvs, __pars->c,__pars->u.key,&len)) != ESP_OK)
          		{
            		return err;
           		}
            }
            else if(__pars->type==PAR_EUI64)
            {
            	if((err = nvs_get_u64(nvs, __pars->c,&__pars->u.ui64par)) != ESP_OK)
          		{
            		return err;
           		}
            }
            else if(__pars->type==PAR_STR)
            {
            	if((err = nvs_get_str(nvs, __pars->c,NULL,&len)) != ESP_OK)
          		{
            		return err;
           		}
            	p=malloc(len);
            	if((err = nvs_get_str(nvs, __pars->c,__pars->u.str,&len)) != ESP_OK)
          		{
            		free(p);
            		return err;
           		}
            }
            __pars++;
        }
    }
    else
    {
        while(__pars->type)
        {
            if(__pars->type==PAR_UI8)
            {
                if ((err = nvs_set_u8(nvs, __pars->c, __pars->u.ui8par)) != ESP_OK)
           		{
                	return err;
           		}
            }
            else if(__pars->type==PAR_UI32)
            {
                if ((err = nvs_set_u32(nvs, __pars->c, __pars->u.ui32par)) != ESP_OK)
           		{
                	nvs_close(nvs);
                	return err;
           		}
            }
            else if(__pars->type==PAR_I32)
            {
                if ((err = nvs_set_i32(nvs, __pars->c, __pars->u.i32par)) != ESP_OK)
           		{
                	return err;
           		}
            }
            else if(__pars->type==PAR_KEY128)
            {
            	if((err = nvs_set_blob(nvs, __pars->c, __pars->u.key,16)) != ESP_OK)
          		{
            		return err;
           		}
            }
            else if(__pars->type==PAR_EUI64)
            {
            	if((err = nvs_set_u64(nvs, __pars->c,__pars->u.ui64par)) != ESP_OK)
          		{
            		return err;
           		}
            }
            else if(__pars->type==PAR_STR)
            {
            	if((err = nvs_set_str(nvs, __pars->c,__pars->u.str)) != ESP_OK)
          		{
            		return err;
           		}
            }
            __pars++;
        }
		if((err=add_uid())!=ESP_OK)
		{
			return err;
		}

		make_deveui();
		nvs_set_u8(nvs,"eeprom",1);

		if((err=nvs_commit(nvs))!=ESP_OK)
		{
			return err;
		}
    }
    return ESP_OK;

}

static void make_deveui(void)
{
    uint8_t deveui[8],joineui[8];
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
    for(uint8_t j=0;j<6;j++)
    {
    	deveui[j+2]=mac[j];
    	joineui[j+2]=mac[j];
    }
    deveui[0]=0;
    deveui[1]=0;
    joineui[0]=0;
    joineui[1]=0;
    nvs_set_blob(nvs, "DEV0EUI",deveui,8);
    nvs_set_blob(nvs, "JOIN0EUI",joineui,8);
}

static void get_uid(uint32_t* uid)
{
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
	*uid=((uint32_t)mac[5])+(((uint32_t)mac[4])<<8)+(((uint32_t)mac[3])<<16)+(((uint32_t)mac[2])<<24);
//	memcpy(uid,&(mac[0]),4);
	ESP_LOGI("get_uid","MAC: %02x %02x %02x %02x %02x %02x %08x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],uid[0]);
}

static esp_err_t add_uid()
{
    esp_err_t err;
    uint32_t uid;
    get_uid(&uid);
    err = nvs_set_u32(nvs, "UID", uid);
    return err;

}


esp_err_t Write_u32_EEPROM(char* key, uint32_t val)
{
	return nvs_set_u32(nvs,key,val);
}

esp_err_t Write_u8_EEPROM(char* key, uint8_t val)
{
	return nvs_set_u8(nvs,key,val);
}

esp_err_t Write_i32_EEPROM(char* key, int32_t val)
{
	return nvs_set_i32(nvs,key,val);
}

esp_err_t Write_key_EEPROM(char* key, uint8_t* appkey)
{
	return nvs_set_blob(nvs,key,appkey,16);
}

esp_err_t Write_eui_EEPROM(char* key, uint8_t* eui)
{
	return nvs_set_u64(nvs,key,*((uint64_t*)eui));
}

esp_err_t Write_str_EEPROM(char* key, char* str)
{
	return nvs_set_str(nvs,key,str);
}

esp_err_t Commit(void)
{
	return nvs_commit(nvs);
}


