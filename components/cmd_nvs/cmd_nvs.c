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
//#include "s2lp_console.h"

/*static const type_str_pair_t type_str_pair[] = {
    { NVS_TYPE_I8, "i8" },
    { NVS_TYPE_U8, "u8" },
    { NVS_TYPE_U16, "u16" },
    { NVS_TYPE_I16, "i16" },
    { NVS_TYPE_U32, "u32" },
    { NVS_TYPE_I32, "i32" },
    { NVS_TYPE_U64, "u64" },
    { NVS_TYPE_I64, "i64" },
    { NVS_TYPE_STR, "str" },
    { NVS_TYPE_BLOB, "blob" },
    { NVS_TYPE_ANY, "any" },
};*/

_param _params[]=
{
		{'F',NVS_TYPE_U32},// base frequency
	    {'M',NVS_TYPE_U8}, // modulation MOD_2FSK
	    {'R',NVS_TYPE_U32}, // datarate
	    {'W',NVS_TYPE_U32}, // bandwidth
	    {'D',NVS_TYPE_U32}, // freq_deviation
	    {'S',NVS_TYPE_U32}, // channel space
	    {'P',NVS_TYPE_I32}, // power
	    {'T',NVS_TYPE_U8}, // transmit/rec
	    {'L',NVS_TYPE_U8}, // use LDO/bypass LDO
	    {'C',NVS_TYPE_I32}, // channel
	    {'E',NVS_TYPE_U32}, // preamble length
	    {'N',NVS_TYPE_U32}, // id
	    {'I',NVS_TYPE_U32}, // interval in seconds
	    {'X',NVS_TYPE_U8}, // repeater
		{'Y',NVS_TYPE_U8}, // JP4 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP4 1 - norm, 0 - alarm
		{'Z',NVS_TYPE_U8}, // JP5 mode, 0-inactive, 1 - change status, 2 - if alarm - non-stop, 0x04 bit: if set JP5 1 - norm, 0 - alarm
		{'G',NVS_TYPE_U8}, // CRC mode - 0x00 -NO CRC, 0x20 - 8 bit, 0x40 - 16 bit 0x8005, 0x60 - 16 bit 0x1021
		{0,0}
};

//static const size_t TYPE_STR_PAIR_SIZE = sizeof(type_str_pair) / sizeof(type_str_pair[0]);
static const char *ARG_TYPE_STR = "type can be: i8, u8, i16, u16 i32, u32 i64, u64, str, blob";
static char current_namespace[16] = "s2lp";
static const char *TAG = "cmd_nvs";

static nvs_type_t key_to_type(const char key)
{

	int i=0;
	while(_params[i].c!=0)
	{
		if(_params[i].c==key) return _params[i].type;
	}
	return NVS_TYPE_ANY;
}

static esp_err_t set_value_in_nvs(const char *key, const char *str_value)
{
    esp_err_t err;
    nvs_handle_t nvs;
    bool range_error = false;

    nvs_type_t type = key_to_type(key[0]);

    if (type == NVS_TYPE_ANY) {
        ESP_LOGE(TAG, "Type for key '%s' is undefined", key);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    err = nvs_open(current_namespace, NVS_READWRITE, &nvs);
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
            ESP_LOGI(TAG, "Value stored under key '%s'", key);
        }
    }

    nvs_close(nvs);
    return err;
}

static esp_err_t get_value_from_nvs(const char *key, int x)
{
    nvs_handle_t nvs;
    esp_err_t err;

    nvs_type_t type = key_to_type(key[0]);

    if (type == NVS_TYPE_ANY) {
        ESP_LOGE(TAG, "Type for key '%s' is undefined", key);
        return ESP_ERR_NVS_TYPE_MISMATCH;
    }

    err = nvs_open(current_namespace, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    if (type == NVS_TYPE_U8) {
        uint8_t value;
        err = nvs_get_u8(nvs, key, &value);
        if (err == ESP_OK) {
            if(!x) printf("%s=%u\n",key,value);
            else printf("%s=0x%02x\n",key,value);
        }
    } else if (type == NVS_TYPE_I32) {
        int32_t value;
        if ((err = nvs_get_i32(nvs, key, &value)) == ESP_OK) {
           if(!x) printf("%s=%d\n", key, value);
           else printf("%s=0x%08x\n", key, value);
        }
    } else if (type == NVS_TYPE_U32) {
        uint32_t value;
        if ((err = nvs_get_u32(nvs, key, &value)) == ESP_OK) {
            if(!x) printf("%s=%u\n", key, value);
            else printf("%s=0x%08x\n", key, value);
        }
    }
    nvs_close(nvs);
    return err;
}

static int list(int x)
{
    nvs_handle_t nvs;
    esp_err_t err;
    err = nvs_open(current_namespace, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        return err;
    }

    nvs_iterator_t it = nvs_entry_find("nvs_key", current_namespace, NVS_TYPE_ANY);
    if (it == NULL) {
        ESP_LOGE(TAG, "No such enty was found");
        return 1;
    }

    do {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        it = nvs_entry_next(it);

        if(!strcmp(info.namespace_name,current_namespace))
        {
            if (info.type == NVS_TYPE_U8) {
                uint8_t value;
                err = nvs_get_u8(nvs, info.key, &value);
                if (err == ESP_OK) {
                    if(!x) printf("%s=%u\n",info.key,value);
                    else printf("%s=0x%02x\n",info.key,value);
                }
            } else if (info.type == NVS_TYPE_I32) {
                int32_t value;
                if ((err = nvs_get_i32(nvs, info.key, &value)) == ESP_OK) {
                   if(!x) printf("%s=%d\n", info.key, value);
                   else printf("%s=0x%08x\n", info.key, value);
                }
            } else if (info.type == NVS_TYPE_U32) {
                uint32_t value;
                if ((err = nvs_get_u32(nvs, info.key, &value)) == ESP_OK) {
                    if(!x) printf("%s=%u\n", info.key, value);
                    else printf("%s=0x%08x\n", info.key, value);
                }
            }
        }
    } while (it != NULL);
    nvs_close(nvs);
    return 0;
}

static int set_value(int argc, char **argv)
{
    ESP_LOGI(TAG,"set %s,%s",argv[0],argv[1]);
	esp_err_t err = set_value_in_nvs(argv[0], argv[1]);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

static int get_value(int argc, char **argv)
{
    ESP_LOGI(TAG,"get %s,%s",argv[0],argv[1]);
    esp_err_t err = get_value_from_nvs(argv[0], 0);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

static int list_entries(int argc, char **argv)
{
    ESP_LOGI(TAG,"list %s,%s",argv[0],argv[1]);
    return list(0);
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
        "Example: list \n",
        .hint = NULL,
        .func = &list_entries,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&set_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_cmd));
    ESP_ERROR_CHECK(esp_console_cmd_register(&list_entries_cmd));
}
