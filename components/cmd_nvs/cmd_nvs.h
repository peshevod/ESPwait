/* Console example — declarations of command registration functions.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "nvs.h"

typedef struct param
{
    char c[16];
    nvs_type_t type;
    char desc[200];
} _param;

typedef enum
{
	RECEIVE_MODE = 0x00,
	TRANSMIT_MODE = 0x01
} tmode_t;

// Register NVS functions
void register_nvs();
void get_uid(uint32_t* uid);
esp_err_t get_value_from_nvs(char *key, int x, char* y,void* value);
esp_err_t set_value_in_nvs(char *key, const char *str_value);
int list_init(void);
int list(int x, char* y, char* value);
esp_err_t add_uid(void);
esp_err_t set_cert(char* key, char* value, int len);
esp_err_t get_certs(void);

#ifdef __cplusplus
}
#endif

