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
#include "cmd_nvs.h"

_param _params[]=
{
		{"F",NVS_TYPE_U32, "Base frequency, Hz"},
	    {"M",NVS_TYPE_U8, "Modulation 0x00: 2-FSK, 0x10: 4-FSK, 0x20: 2-GFSK BT=1, 0x30: 4-GFSK BT=1, 0x50: ASK/OOK, 0x60: polar, 0x70: no mod(CW), 0xa0: 2-GFSK BT=0.5, 0xb0: 4-GFSK BT=0.5"},
	    {"R",NVS_TYPE_U32,"Datarate bit/s"},
	    {"W",NVS_TYPE_U32,"Bandwidth, Hz"},
	    {"D",NVS_TYPE_U32, "Frequency deviation, Hz"},
	    {"S",NVS_TYPE_U32, "Channel spacing, Hz"},
		{"P",NVS_TYPE_I32, "Power, dbm"},
	    {"T",NVS_TYPE_U8, "Mode 1: transmit, 0: receive"},
	    {"L",NVS_TYPE_U8, "LDO bypass 1: yes, 0 no"},
	    {"C",NVS_TYPE_I32, "Channel number"},
	    {"E",NVS_TYPE_U32, "Preamble length - number of 01 or 10 sequencies. If Preamble length == 0 then continuous transmit with PN9 source"},
	    {"UID",NVS_TYPE_U32, "UID - is low 4 bytes of MAC address"},
	    {"I",NVS_TYPE_U32, "Interval between actions (trans or rec), sec."},
	    {"X",NVS_TYPE_U8, "Number of repeated messages in trans mode"},
		{"Y",NVS_TYPE_U8, "Sensor JP4 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP4 1 - norm, 0 - alarm"},
		{"Z",NVS_TYPE_U8, "Sensor JP5 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP5 1 - norm, 0 - alarm"},
		{"G",NVS_TYPE_U8, "CRC mode - 0x00 -NO CRC, 0x20 - 8 bit, 0x40 - 16 bit 0x8005, 0x60 - 16 bit 0x1021"},
		{"V",NVS_TYPE_I32, "Sensitivity (dbm)"},
		{"SSID",NVS_TYPE_STR,"SSID"},
		{"PASSWD",NVS_TYPE_STR,"Password for ssid"},
		{"MD5ROOT",NVS_TYPE_STR,"MD5 sum for root certificate"},
		{"MD5CERT",NVS_TYPE_STR,"MD5 sum for certificate"},
		{"MD5KEY",NVS_TYPE_STR,"MD5 sum for key"},
		{"ROOT",NVS_TYPE_BLOB,"root certificate"},
		{"CERT",NVS_TYPE_BLOB,"certificate"},
		{"KEY",NVS_TYPE_BLOB,"private key"},
		{"",0,""}
};

_par_t _pars[]=
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
	{PAR_STR,"SSID",{.str="Paritet"},"SSID",,VISIBLE},
	{PAR_STR,"PASSWD",{.str="narrowcartoon617"},"Password for ssid",VISIBLE},
	{PAR_MD5,"MD5ROOT",{.str=NULL},"MD5 sum for root certificate",VISIBLE},
	{PAR_MD5,"MD5CERT",{.str=NULL},"MD5 sum for certificate",VISIBLE},
	{PAR_MD5,"MD5KEY",{.str=NULL},"MD5 sum for key",VISIBLE},
	{PAR_CERT,"ROOT",{.blob=NULL},"root certificate",VISIBLE},
	{PAR_CERT,"CERT",{.blob=NULL},"certificate",VISIBLE},
	{PAR_CERT,"KEY",{.blob=NULL},"private key",VISIBLE},
	{0,NULL,{0},NULL}
}

extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint16_t aws_root_ca_pem_length asm("aws_root_ca_pem_length");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint16_t certificate_pem_crt_length asm("certificate_pem_crt_length");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");
extern const uint16_t private_pem_key_length asm("private_pem_key_length");

nvs_handle_t nvs;
uint8_t s2lp_console_ex=0;

static char eeprom_namespace[16] = "eeprom";
static char eeprom_partition[16] = "eeprom";
static const char *TAG = "cmd_nvs";
static char key0[17];
char* pKey;
char* pRoot;
char* pCert;

int write_cert_to_nvs(char* cert, char* data, int len,char* str_md5)
{
	char key_name[16];
	unsigned char md5[16];
	if(!strcmp(cert,"KEY") || !strcmp(cert,"ROOT") || !strcmp(cert,"CERT") )
	{
		if(mbedtls_md5_ret((const unsigned char*)data,len,md5)) return -1;
		sprintf(str_md5,"%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",md5[0],md5[1],md5[2],md5[3],md5[4],md5[5],md5[6],md5[7],md5[8],md5[9],md5[10],md5[11],md5[12],md5[13],md5[14],md5[15]);
		strcpy(key_name,"MD5");
		strcat(key_name,cert);
		if(set_cert(cert,data,len)==ESP_OK)
		{
			ESP_LOGI(__func__,"certificate type=%s length %d successfully written to flash",cert,len-1);
			if(set_value_in_nvs(key_name,str_md5)!=ESP_OK) return -2;
			ESP_LOGI(__func__,"certificate type=%s %s = %s",cert,key_name,str_md5);
		}
		if(!strcmp(cert,"KEY")) pKey=data;
		else if(!strcmp(cert,"ROOT")) pRoot=data;
		else if(!strcmp(cert,"CERT")) pCert=data;
		return 0;
	}
	ESP_LOGI(__func__,"wrong cert type");
	return -3;
}




void Sync_EEPROM(void)
{
    _par_t* __pars=_pars;
    int i=1;
    esp_err_t err;
    nvs_handle_t nvs;
    uint8_t v8;
    size_t len;
    void* p;
	char str_md5[40];

	init_uart0();
	initialize_nvs();

	err = nvs_open_from_partition(eeprom_partition, eeprom_namespace, NVS_READWRITE, &nvs);
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
            	p=malloc(16);
            	if((err = nvs_get_blob(nvs, __pars->c,(uint8_t*)p,&len)) != ESP_OK)
          		{
            		free(p);
            		p=NULL;
            		return err;
           		}
            	__pars->c.u.key=(uint8_t*)p;
            }
            else if(__pars->type==PAR_EUI64)
            {
            	p=malloc(8);
            	if((err = nvs_get_blob(nvs, __pars->c,(uint8_t*)p,&len)) != ESP_OK)
          		{
            		free(p);
            		p=NULL;
            		return err;
           		}
            	__pars->c.u.eui=(uint8_t*)p;
            }
            else if(__pars->type==PAR_STR)
            {
            	err = nvs_get_str(nvs, __pars->c,NULL,&len);
            	if(err!=ESP_OK)
            	{
            		return err;
            	}
            	if(len!=0)
            	{
            		p=malloc(len);
            		if((err = nvs_get_str(nvs, __pars->c,(char*)p,&len)) != ESP_OK) return err;
            		__pars->c.u.str=(char*)p;
            	}
            	else
            	{
            		__pars->c.u.str=NULL;
            	}
            }
            __pars++;
        }
        get_certs();
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
            	if((err = nvs_set_blob(nvs, __pars->c,__pars->u.eui,8)) != ESP_OK)
          		{
            		return err;
           		}
            }
            else if(__pars->type==PAR_STR)
            {
            	if((err = nvs_set_str(nvs, __pars->c,__pars->u.str)) != ESP_OK )
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

		if(write_cert_to_nvs("CERT",(char*)certificate_pem_crt_start, certificate_pem_crt_end-certificate_pem_crt_start, str_md5))
			ESP_LOGE(__func__,"certificate type=CERT length %d failed write to flash",certificate_pem_crt_end-certificate_pem_crt_start-1);
		else pCert=(char*)certificate_pem_crt_start;
		if(write_cert_to_nvs("ROOT",(char*)aws_root_ca_pem_start, aws_root_ca_pem_end-aws_root_ca_pem_start, str_md5))
			ESP_LOGE(__func__,"certificate type=ROOT length %d failed write to flash",aws_root_ca_pem_end-aws_root_ca_pem_start-1);
		else pRoot=(char*)aws_root_ca_pem_start;
		if(write_cert_to_nvs("KEY",(char*)private_pem_key_start, private_pem_key_end-private_pem_key_start, str_md5))
			ESP_LOGE(__func__,"certificate type=KEY length %d failed write to flash", private_pem_key_end-private_pem_key_start-1);
		else pKey=(char*)private_pem_key_start;

		if((err=nvs_commit(nvs))!=ESP_OK)
		{
			return err;
		}
    }
    return ESP_OK;

}

void make_deveui(void)
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
    nvs_set_blob(nvs, "DEV0EUI",deveui,8)
    nvs_set_blob(nvs, "JOIN0EUI",joineui,8)
}

void get_uid(uint32_t* uid)
{
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
	*uid=((uint32_t)mac[5])+(((uint32_t)mac[4])<<8)+(((uint32_t)mac[3])<<16)+(((uint32_t)mac[2])<<24);
//	memcpy(uid,&(mac[0]),4);
	ESP_LOGI("get_uid","MAC: %02x %02x %02x %02x %02x %02x %08x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],uid[0]);
}

static int key_to_type(char* key)
{

	int i=0;
    while (key[i]) {
    	key0[i] = toupper((unsigned char) key[i]);
    	i++;
    }
    key0[i]=0;
    i=0;
	while(_pars[i].type!=0)
	{
		if(!strcmp(key0,_params[i].c)) return i;
		i++;
	}
	return -1;
}


esp_err_t add_uid()
{
    esp_err_t err;
    uint32_t uid;
    get_uid(&uid);
    err = nvs_set_u32(nvs, "UID", uid);
    return err;

}

esp_err_t set_cert(char* key, char* value, int len)
{
    esp_err_t err;

    err=nvs_set_blob(nvs, key, value, len);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}


esp_err_t get_certs()
{
    esp_err_t err;
    size_t len;
    err=nvs_get_blob(nvs, "KEY", NULL, &len);
    if (err == ESP_OK) {
    	pKey=(char*)malloc(len+1);
       	err=nvs_get_blob(nvs, "KEY", pKey, &len);
       	if (err != ESP_OK) {
       		free(pKey);
       		pKey=NULL;
       	}
       	pKey[len]=0;
    } else pKey=NULL;

    err=nvs_get_blob(nvs, "ROOT", NULL, &len);
    if (err == ESP_OK) {
    	pRoot=(char*)malloc(len+1);
       	err=nvs_get_blob(nvs, "ROOT", pRoot, &len);
       	if (err != ESP_OK) {
       		free(pRoot);
       		pRoot=NULL;
       	}
       	pRoot[len]=0;
    } else pRoot=NULL;

    err=nvs_get_blob(nvs, "CERT", NULL, &len);
    if (err == ESP_OK) {
    	pCert=(char*)malloc(len+1);
       	err=nvs_get_blob(nvs, "CERT", pCert, &len);
       	if (err != ESP_OK) {
       		free(pCert);
       		pCert=NULL;
       	}
       	pCert[len]=0;
    } else pCert=NULL;

    return ESP_OK;

}

esp_err_t set_value_in_nvs(char *key, const char *str_value)
{
    esp_err_t err;

    bool range_error = false;

    int itab=key_to_type(key);
    if (itab==-1) {
        ESP_LOGE(TAG, "Key '%s' is undefined", key);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    par_type_t type = _params[itab].type;

    if (type == NVS_TYPE_U8) {
        uint32_t value = strtoul(str_value, NULL, 0);
        if (value > UINT8_MAX || errno == ERANGE) {
            range_error = true;
        } else {
            err = nvs_set_u8(nvs, key0, (uint8_t)value);
        }
    } else if (type == NVS_TYPE_I32) {
        int32_t value = strtol(str_value, NULL, 0);
        if (errno != ERANGE) {
            err = nvs_set_i32(nvs, key0, value);
        }
    } else if (type == NVS_TYPE_U32) {
        uint32_t value = strtoul(str_value, NULL, 0);
        if (errno != ERANGE) {
            err = nvs_set_u32(nvs, key0, value);
        }
    }  else if (type == NVS_TYPE_STR) {
	   err = nvs_set_str(nvs, key0, str_value);
    }
    else
    {
    	return err;
    }


    if (range_error || errno == ERANGE) {
        return ESP_ERR_NVS_VALUE_TOO_LONG;
    }

    if (err == ESP_OK) {
        err = nvs_commit(nvs);
//        if (err == ESP_OK) {
//            ESP_LOGI(TAG, "Value stored under key '%s'", key);
//        }
    }

    return err;
}

esp_err_t get_value_from_nvs(char *key, int x, char* y, void* value)
{
    nvs_handle_t nvs;
    esp_err_t err=ESP_OK;
    size_t len;

    int itab=key_to_type(key);
    if (itab==-1) {
        ESP_LOGE(TAG, "Key '%s' is undefined", key);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    nvs_type_t type = _params[itab].type;

    err = nvs_open_from_partition(s2lp_partition, s2lp_namespace, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }


    if (type == NVS_TYPE_U8) {
        if ((err = nvs_get_u8(nvs, key0, (uint8_t*)value)) == ESP_OK) {
           	if(y!=NULL)
           	{
           		if(!x) sprintf(y,"%s=%u    %s\n",key0,((uint8_t*)value)[0],_params[itab].desc);
           		else sprintf(y,"%s=0x%02X    %s\n",key0,((uint8_t*)value)[0],_params[itab].desc);
           	}
        }
    } else if (type == NVS_TYPE_I32) {
        if ((err = nvs_get_i32(nvs, key0, (int32_t*)value)) == ESP_OK)
        {
           	if(y!=NULL)
           	{
           		if(!x) sprintf(y,"%s=%d    %s\n", key0, *((int32_t*)value),_params[itab].desc);
           		else sprintf(y,"%s=0x%08X    %s\n", key0, *((int32_t*)value),_params[itab].desc);
           	}
        }
    } else if (type == NVS_TYPE_U32) {
        if ((err = nvs_get_u32(nvs, key0, (uint32_t*)value)) == ESP_OK)
        {
           	if(y!=NULL)
           	{
           		if(!x) sprintf(y,"%s=%d    %s\n", key0, *((uint32_t*)value),_params[itab].desc);
           		else sprintf(y,"%s=0x%08X    %s\n", key0, *((uint32_t*)value),_params[itab].desc);
           	}
        }
    } else if (type == NVS_TYPE_STR) {
    	if ((err = nvs_get_str(nvs, key0, (char*)value,&len)) == ESP_OK) {
ESP_LOGI(__func__,"Key=%s type=%d, val=%s",key0,type, (char*)value);
    		if(y!=NULL)	sprintf(y,"%s=%s    %s\n", key0, (char*)value,_params[itab].desc);
		} else ESP_LOGI(__func__,"Failed Key=%s, type=%d, err=%s, nerr=%d",key0,type,esp_err_to_name(err),err);
    }
    else
    {
    	if(y!=NULL) y[0]=0;
    	((char*)value)[0]=0;
    }
    nvs_close(nvs);
    return err;
}


static nvs_iterator_t it;
static nvs_handle_t nvs0;

int list_init()
{
    esp_err_t err;
    err = nvs_open_from_partition(s2lp_partition, s2lp_namespace, NVS_READONLY, &nvs0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return err;
    }
//    ESP_LOGI(TAG, "Open namespace %s done",current_namespace);

    it = nvs_entry_find(s2lp_partition, s2lp_namespace, NVS_TYPE_ANY);
    if (it == NULL) {
        ESP_LOGE(TAG, "No such enty was found");
        return -1;
    }
    return 0;

}


int list(int x, char* y, char* value)
{
    esp_err_t err;
    nvs_entry_info_t info;
    nvs_entry_info(it, &info);
    it = nvs_entry_next(it);
    int itab=key_to_type(info.key);
    nvs_type_t type = _params[itab].type;

    if (type == NVS_TYPE_U8) {
        if ((err = nvs_get_u8(nvs0, key0, (uint8_t*)value)) == ESP_OK) {
           	if(y!=NULL)
           	{
           		if(!x) sprintf(y,"%s=%u    %s\n",key0,((uint8_t*)value)[0],_params[itab].desc);
           		else sprintf(y,"%s=0x%02X    %s\n",key0,((uint8_t*)value)[0],_params[itab].desc);
           	}
        }
    } else if (type == NVS_TYPE_I32) {
        if ((err = nvs_get_i32(nvs0, key0, (int32_t*)value)) == ESP_OK)
        {
           	if(y!=NULL)
           	{
           		if(!x) sprintf(y,"%s=%d    %s\n", key0, ((int32_t*)value)[0],_params[itab].desc);
           		else sprintf(y,"%s=0x%08X    %s\n", key0, ((int32_t*)value)[0],_params[itab].desc);
           	}
        }
    } else if (type == NVS_TYPE_U32) {
        if ((err = nvs_get_u32(nvs0, key0, (uint32_t*)value)) == ESP_OK)
        {
           	if(y!=NULL)
           	{
           		if(!x) sprintf(y,"%s=%d    %s\n", key0, *((uint32_t*)value),_params[itab].desc);
           		else sprintf(y,"%s=0x%08X    %s\n", key0, *((uint32_t*)value),_params[itab].desc);
           	}
        }
    } else if (type == NVS_TYPE_STR) {
      	size_t len;
       	if ((err = nvs_get_str(nvs0, key0, (char*)value,&len)) == ESP_OK) {
       		value[len]=0;
       		if(y!=NULL)	sprintf(y,"%s=%s    %s\n", key0, (char*)value,_params[itab].desc);
   		}
    }
    else
    {
    	y[0]=0;
    	value[0]=0;
    }

    if(it!=NULL) return 1;
    nvs_close(nvs0);
    return 0;
}




static int set_value(int argc, char **argv)
{
//    ESP_LOGI(TAG,"set %s,%s",argv[0],argv[1]);
	esp_err_t err = set_value_in_nvs(argv[1], argv[2]);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

static int get_value(int argc, char **argv)
{
//    ESP_LOGI(TAG,"get %s,%s",argv[0],argv[1]);
	esp_err_t err;
	char value[64];
	char s[256];
	if(argc==3 && argv[1][0]=='-' && argv[1][1]=='x') err = get_value_from_nvs(argv[2], 1,s,value);
	else err = get_value_from_nvs(argv[1], 0,s,value);
	printf("%s\n",s);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

static int list_entries(int argc, char **argv)
{
//    ESP_LOGI(TAG,"list %d %s",argc,argv[0]);
	int rc;
	int x=0;
	char y[256],value[64];
	if(argc==2 && argv[1][0]=='-' && argv[1][1]=='x') x=1;
    else x=0;
	list_init();
	do
	{
		rc=list(x,y,value);
		printf("%s",y);
	} while(rc==1);
	return rc;
}

static int exit_from_console(int argc, char **argv)
{
	s2lp_console_ex=1;
	return 0;
}

void register_nvs()
{
    const esp_console_cmd_t set_cmd = {
        .command = "set",
        .help = "Set key-value pair in selected namespace.\n"
        "Examples:\n"
        "set -k 123 \n",
        .hint = NULL,
        .func = &set_value,
        .argtable = NULL
    };

    const esp_console_cmd_t get_cmd = {
        .command = "get",
        .help = "Get key-value pair from selected namespace. \n"
        "Example: get VarName",
        .hint = NULL,
        .func = &get_value,
        .argtable = NULL
    };

    const esp_console_cmd_t list_entries_cmd = {
        .command = "list",
        .help = "List stored key-value pairs stored in NVS."
        "Example: list -x \n",
        .hint = NULL,
        .func = &list_entries,
        .argtable = NULL
    };

    const esp_console_cmd_t exit_cmd = {
        .command = "exit",
        .help = "Exit from console",
        .hint = NULL,
        .func = &exit_from_console,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&list_entries_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&exit_cmd));
}
