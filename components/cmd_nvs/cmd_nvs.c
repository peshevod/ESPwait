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
	    {"M",NVS_TYPE_U8, "Modulation 0x00: 2-FSK, 0x10: 4-FSK, 0x20: 2-GFSK BT=1, 0x30: 4-GFSK BT=1, 0x50: ASK/OOK, 0x60: polar, 0x70: no mod, 0xa0: 2-GFSK BT=0.5, 0xb0: 4-GFSK BT=0.5"},
	    {"R",NVS_TYPE_U32,"Datarate bit/s"},
	    {"W",NVS_TYPE_U32,"Bandwidth, Hz"},
	    {"D",NVS_TYPE_U32, "Frequency deviation, Hz"},
	    {"S",NVS_TYPE_U32, "Channel spacing, Hz"},
		{"P",NVS_TYPE_I32, "Power, dbm"},
	    {"T",NVS_TYPE_U8, "Mode 1: transmit, 0: receive"},
	    {"L",NVS_TYPE_U8, "LDO bypass 1: yes, 0 no"},
	    {"C",NVS_TYPE_I32, "Channel number"},
	    {"E",NVS_TYPE_U32, "Preamble length - number of 01 or 10 sequencies"},
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

uint8_t s2lp_console_ex=0;

static char s2lp_namespace[16] = "s2lp";
static char s2lp_partition[16] = "s2lp";
static const char *TAG = "cmd_nvs";
static char key0[17];

void get_uid(uint32_t* uid)
{
	uint8_t mac[6];
	ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));
	memcpy(uid,&(mac[2]),4);
//	ESP_LOGI("get_uid","MAC: %02x %02x %02x %02x %02x %02x %08x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],uid[0]);
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
	while(_params[i].type!=0)
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
    nvs_handle_t nvs;
    get_uid(&uid);
    err = nvs_open_from_partition(s2lp_partition, s2lp_namespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u32(nvs, "UID", uid);
    if (err == ESP_OK) {
        err = nvs_commit(nvs);
    }

    nvs_close(nvs);
    return err;

}

esp_err_t set_cert(char* key, char* value, int len)
{
    esp_err_t err;
    nvs_handle_t nvs;
    err = nvs_open_from_partition(s2lp_partition, s2lp_namespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    err=nvs_set_blob(nvs, key, value, len);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_commit(nvs);
    if (err != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

esp_err_t set_value_in_nvs(char *key, const char *str_value)
{
    esp_err_t err;
    nvs_handle_t nvs;
    bool range_error = false;

    int itab=key_to_type(key);
    if (itab==-1) {
        ESP_LOGE(TAG, "Key '%s' is undefined", key);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    nvs_type_t type = _params[itab].type;

    err = nvs_open_from_partition(s2lp_partition, s2lp_namespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

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
    	nvs_close(nvs);
    	return err;
    }


    if (range_error || errno == ERANGE) {
        nvs_close(nvs);
        return ESP_ERR_NVS_VALUE_TOO_LONG;
    }

    if (err == ESP_OK) {
        err = nvs_commit(nvs);
//        if (err == ESP_OK) {
//            ESP_LOGI(TAG, "Value stored under key '%s'", key);
//        }
    }

    nvs_close(nvs);
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
    		((char*)value)[len]=0;
    		if(y!=NULL)	sprintf(y,"%s=%s    %s\n", key0, (char*)value,_params[itab].desc);
		}
    }
    else
    {
    	y[0]=0;
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
