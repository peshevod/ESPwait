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

#include "esp_err.h"
#include "eui.h"

#define VISIBLE 1
#define HIDDEN  0

#define MAX_EEPROM_RECORDS 64

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

typedef union
{
	uint64_t u64;
	uint32_t u32[2];
	uint16_t u16[4];
	uint8_t u8[8];
	struct
	{
		uint32_t u32low;
		struct
		{
			uint16_t DevNonce;
			struct
			{
				uint8_t u8low;
				uint8_t type;
			} x16;

		} x32;
	} x64;
} Record_t;

// Register NVS functions
esp_err_t Sync_EEPROM(void);
esp_err_t Write_u32_params(char* key, uint32_t val);
esp_err_t Write_u8_params(char* key, uint8_t val);
esp_err_t Write_i32_params(char* key, int32_t val);
esp_err_t Write_key_params(char* key, uint8_t* appkey);
esp_err_t Write_eui_params(char* key, uint8_t* eui);
esp_err_t Write_str_params(char* key, char* str);
esp_err_t Commit_params(void);
esp_err_t get_Eui(uint8_t n, GenericEui_t* deveui);
esp_err_t put_Eui(uint8_t n, GenericEui_t* deveui);
uint8_t get_EUI_type(uint8_t n);
void set_EUI_type(uint8_t n);
void clear_EUI_type(uint8_t n);
uint16_t get_DevNonce(uint8_t n);
void put_DevNonce(uint8_t n, uint16_t DevNonce);
uint8_t get_eui_numbers(void);
uint8_t increase_eui_numbers(void);
esp_err_t erase_EEPROM_Data(void);
esp_err_t Commit_deveui(void);


#ifdef __cplusplus
}
#endif

