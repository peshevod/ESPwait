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

typedef struct param
{
    char c;
    nvs_type_t type;
    char desc[200];
} _param;

// Register NVS functions
void register_nvs();
void get_uid(uint32_t* uid);

#ifdef __cplusplus
}
#endif

