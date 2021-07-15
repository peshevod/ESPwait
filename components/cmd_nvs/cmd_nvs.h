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

#define VISIBLE 1
#define HIDDEN  0

typedef enum
{
    PAR_UI32=1,
    PAR_I32,
    PAR_UI8,
    PAR_KEY128,
    PAR_EUI64,
    PAR_STR
//    PAR_MD5,
//	PAR_CERT
} par_type_t;

typedef struct par
{
    par_type_t type;
    char* c;
    union
    {
        uint32_t ui32par;
        int32_t i32par;
        uint8_t ui8par;
        uint8_t key[16];
        uint8_t eui[8];
        char* str;
        uint64_t ui64par;
    } u;
    char* d;
    uint8_t visible;
} _par_t;


typedef enum
{
	RECEIVE_MODE = 0x00,
	TRANSMIT_MODE = 0x01
} tmode_t;

// Register NVS functions
esp_err_t Sync_EEPROM(void);
esp_err_t Write_u32_EEPROM(char* key, uint32_t val);
esp_err_t Write_u8_EEPROM(char* key, uint8_t val);
esp_err_t Write_i32_EEPROM(char* key, int32_t val);
esp_err_t Write_key_EEPROM(char* key, uint8_t* appkey);
esp_err_t Write_eui_EEPROM(char* key, uint8_t* eui);
esp_err_t Write_str_EEPROM(char* key, char* str);


#ifdef __cplusplus
}
#endif

