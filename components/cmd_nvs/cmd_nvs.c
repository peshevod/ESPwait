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
#include <inttypes.h>
#include "esp_console.h"
#include "esp_log.h"
#include "argtable3/argtable3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "nvs.h"
#include "cmd_nvs.h"

_param _params[]=
{
		{'F',NVS_TYPE_U32, "Base frequency, Hz"},
	    {'M',NVS_TYPE_U8, "Modulation 0x00: 2-FSK, 0x10: 4-FSK, 0x20: 2-GFSK BT=1, 0x30: 4-GFSK BT=1, 0x50: ASK/OOK, 0x60: polar, 0x70: no mod, 0xa0: 2-GFSK BT=0.5, 0xb0: 4-GFSK BT=0.5"},
	    {'R',NVS_TYPE_U32,"Datarate bit/s"},
	    {'W',NVS_TYPE_U32,"Bandwidth, Hz"},
	    {'D',NVS_TYPE_U32, "Frequency deviation, Hz"},
	    {'S',NVS_TYPE_U32, "Channel spacing, Hz"},
		{'P',NVS_TYPE_I32, "Power, dbm"},
	    {'T',NVS_TYPE_U8, "Mode 1: transmit, 0: receive"},
	    {'L',NVS_TYPE_U8, "LDO bypass 1: yes, 0 no"},
	    {'C',NVS_TYPE_I32, "Channel number"},
	    {'E',NVS_TYPE_U32, "Preamble length - number of 01 or 10 sequencies"},
	    {'N',NVS_TYPE_U32, "ID - not used, now id is low 4 bytes of MAC address"},
	    {'I',NVS_TYPE_U32, "Interval between actions (trans or rec), sec."},
	    {'X',NVS_TYPE_U8, "Number of repeated messages in trans mode"},
		{'Y',NVS_TYPE_U8, "Sensor JP4 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP4 1 - norm, 0 - alarm"},
		{'Z',NVS_TYPE_U8, "Sensor JP5 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP5 1 - norm, 0 - alarm"},
		{'G',NVS_TYPE_U8, "CRC mode - 0x00 -NO CRC, 0x20 - 8 bit, 0x40 - 16 bit 0x8005, 0x60 - 16 bit 0x1021"},
		{0,0,""}
};

int ex=0;

//static const size_t TYPE_STR_PAIR_SIZE = sizeof(type_str_pair) / sizeof(type_str_pair[0]);
//static const char *ARG_TYPE_STR = "type can be: i8, u8, i16, u16 i32, u32 i64, u64, str, blob";
static char current_namespace[16] = "s2lp";
static const char *TAG = "cmd_nvs";

static int key_to_type(char* key)
{

	int i=0;
	if(key[0]<=0x7A && key[0]>=0x61) key[0]-=0x20;
	while(_params[i].c!=0)
	{
		if(_params[i].c==key[0]) return i;
		i++;
	}
	return -1;
}

static esp_err_t set_value_in_nvs(const char *key, const char *str_value)
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

    err = nvs_open_from_partition("s2lp", current_namespace, NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    if (type == NVS_TYPE_U8) {
        uint32_t value = strtoul(str_value, NULL, 0);
        if (value > UINT8_MAX || errno == ERANGE) {
            range_error = true;
        } else {
            err = nvs_set_u8(nvs, key, (uint8_t)value);
        }
    } else if (type == NVS_TYPE_I32) {
        int32_t value = strtol(str_value, NULL, 0);
        if (errno != ERANGE) {
            err = nvs_set_i32(nvs, key, value);
        }
    } else if (type == NVS_TYPE_U32) {
        uint32_t value = strtoul(str_value, NULL, 0);
        if (errno != ERANGE) {
            err = nvs_set_u32(nvs, key, value);
        }
    }

    if (range_error || errno == ERANGE) {
        nvs_close(nvs);
        return ESP_ERR_NVS_VALUE_TOO_LONG;
    }

    if (err == ESP_OK) {
        err = nvs_commit(nvs);
        if (err == ESP_OK) {
//            ESP_LOGI(TAG, "Value stored under key '%s'", key);
        }
    }

    nvs_close(nvs);
    return err;
}

static esp_err_t get_value_from_nvs(const char *key, int x)
{
    nvs_handle_t nvs;
    esp_err_t err;

    int itab=key_to_type(key);
    if (itab==-1) {
        ESP_LOGE(TAG, "Key '%s' is undefined", key);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    nvs_type_t type = _params[itab].type;

    err = nvs_open_from_partition("s2lp", current_namespace, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    if (type == NVS_TYPE_U8) {
        uint8_t value;
        err = nvs_get_u8(nvs, key, &value);
        if (err == ESP_OK) {
            if(!x) printf("%s=%u    %s\n",key,value,_params[itab].desc);
            else printf("%s=0x%02X    %s\n",key,value,_params[itab].desc);
        }
    } else if (type == NVS_TYPE_I32) {
        int32_t value;
        if ((err = nvs_get_i32(nvs, key, &value)) == ESP_OK) {
           if(!x) printf("%s=%d    %s\n", key, value,_params[itab].desc);
           else printf("%s=0x%08X    %s\n", key, value,_params[itab].desc);
        }
    } else if (type == NVS_TYPE_U32) {
        uint32_t value;
        if ((err = nvs_get_u32(nvs, key, &value)) == ESP_OK) {
            if(!x) printf("%s=%u    %s\n", key, value,_params[itab].desc);
            else printf("%s=0x%08X    %s\n", key, value,_params[itab].desc);
        }
    }
    nvs_close(nvs);
    return err;
}

static int list(int x)
{
    nvs_handle_t nvs;
    esp_err_t err;
//    ESP_LOGI(TAG, "Enter in list");
    err = nvs_open_from_partition("s2lp", current_namespace, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return err;
    }
//    ESP_LOGI(TAG, "Open namespace %s done",current_namespace);

    nvs_iterator_t it = nvs_entry_find("s2lp", current_namespace, NVS_TYPE_ANY);
    if (it == NULL) {
        ESP_LOGE(TAG, "No such enty was found");
        return 1;
    }

    do {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        it = nvs_entry_next(it);
        int itab=key_to_type(info.key);

        if(!strcmp(info.namespace_name,current_namespace))
        {
            if (info.type == NVS_TYPE_U8) {
                uint8_t value;
                err = nvs_get_u8(nvs, info.key, &value);
                if (err == ESP_OK) {
                    if(!x) printf("%s=%u    %s\n",info.key,value,_params[itab].desc);
                    else printf("%s=0x%02X    %s\n",info.key,value,_params[itab].desc);
                }
            } else if (info.type == NVS_TYPE_I32) {
                int32_t value;
                if ((err = nvs_get_i32(nvs, info.key, &value)) == ESP_OK) {
                   if(!x) printf("%s=%d    %s\n", info.key, value,_params[itab].desc);
                   else printf("%s=0x%08X    %s\n", info.key, value,_params[itab].desc);
                }
            } else if (info.type == NVS_TYPE_U32) {
                uint32_t value;
                if ((err = nvs_get_u32(nvs, info.key, &value)) == ESP_OK) {
                    if(!x) printf("%s=%u    %s\n", info.key, value,_params[itab].desc);
                    else printf("%s=0x%08X    %s\n", info.key, value,_params[itab].desc);
                }
            }
        }
    } while (it != NULL);
    nvs_close(nvs);
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
	if(argc==3 && argv[1][0]=='-' && argv[1][1]=='x') err = get_value_from_nvs(argv[2], 1);
	else err = get_value_from_nvs(argv[1], 0);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

static int list_entries(int argc, char **argv)
{
    ESP_LOGI(TAG,"list %d %s",argc,argv[0]);
    if(argc==2 && argv[1][0]=='-' && argv[1][1]=='x') return list(1);
    else return list(0);
}

static int exit_from_console(int argc, char **argv)
{
	ex=1;
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
