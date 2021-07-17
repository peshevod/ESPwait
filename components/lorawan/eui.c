/*
 * eui.c
 *
 *  Created on: 16 июл. 2021 г.
 *      Author: ilya_000
 */

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include "esp_log.h"
#include "esp_console.h"
#include "cmd_nvs.h"
#include "lorawan_types.h"
#include "esp_err.h"
#include "shell.h"
#include "esp_system.h"

Profile_t devices[64];
char TAG[]={"eui.c"};

uint8_t euicmpnz(GenericEui_t* eui)
{
    if(eui->eui!=0) return 1;
    return 0;
}

uint8_t euicmp(GenericEui_t* eui1, GenericEui_t* eui2)
{
    if(eui1->eui!=eui2->eui) return 1;
    return 0;
}

uint8_t euicmpr(GenericEui_t* eui1, GenericEui_t* eui2)
{
    for(uint8_t j=0;j<8;j++) if( eui1->buffer[7-j] != eui2->buffer[j] ) return 1;
    return 0;
}

uint8_t selectJoinServer(void* joinServer)
{
    char joinName[9];
    uint8_t eui_numbers;
    GenericEui_t join_eeprom;
    uint8_t jsnumber,js=0,found=0,jlast=0;
//    printVar("before EEPROM_types=",PAR_UI32,&EEPROM_types,true,true);
    set_s("JSNUMBER",&jsnumber);
    strcpy(joinName,"JOIN0EUI");
    for(uint8_t j=1;j<4;j++)
    {
        joinName[4]=0x30 + j;
        set_s(joinName,&(((Profile_t*)joinServer)->Eui));
//         printVar("test j=",PAR_UI8,&j,false,false);
//         printVar(" Eui=",PAR_EUI64,&(joinServer->Eui),true,true);
        jlast=j;
        if(euicmpnz(&(((Profile_t*)joinServer)->Eui)))
        {
            found=0;
            eui_numbers=get_eui_numbers();
            for(uint8_t k=0;k<eui_numbers;k++)
            {
                if((get_Eui(k,&join_eeprom))==ESP_OK)
                {
                     if(!euicmp(&(((Profile_t*)joinServer)->Eui),&(join_eeprom)) && get_EUI_type(k))
                     {
//                         printVar("found k=",PAR_UI8,&k,false,false);
//                         printVar(" Eui=",PAR_EUI64,&(join_eeprom.Eui),true,true);
                         found=1;
                         if(jsnumber==j)
                         {
                             js=k;
                             ((Profile_t*)joinServer)->DevNonce=get_DevNonce(js);
//                             printVar("selected k=",PAR_UI8,&k,false,false);
//                             printVar(" Eui=",PAR_EUI64,&(join_eeprom.Eui),true,true);
                         };
                         break;
                     }
                }
                else return 0;
            }
            if(!found)
            {
                put_Eui(eui_numbers,&(((Profile_t*)joinServer)->Eui));
                ((Profile_t*)joinServer)->DevNonce=0;
                put_DevNonce(eui_numbers, ((Profile_t*)joinServer)->DevNonce);
                set_EUI_type(eui_numbers);
//                 printVar("write kfree=",PAR_UI8,&kfree,false,false);
//                 printVar(" Eui=",PAR_EUI64,&(joinServer->Eui),true,true);
                if(jsnumber==j)
                {
                    js=eui_numbers;
//                     printVar("selected kfree=",PAR_UI8,&kfree,false,false);
//                     printVar(" Eui=",PAR_EUI64,&(joinServer->Eui),true,true);
                }
                eui_numbers=increase_eui_numbers();
                Commit_deveui();
            }
        }
    }
    if(jlast!=jsnumber)
    {
        get_Eui(js,&(((Profile_t*)joinServer)->Eui));
        ((Profile_t*)joinServer)->DevNonce=get_DevNonce(js);
    }
    ((Profile_t*)joinServer)->js=js;
    ESP_LOGI("eui.c","Selected JoinServer js=%d Eui=%016llX\n",js,((Profile_t*)joinServer)->Eui.eui);
    return js;
}

uint8_t fill_devices(void)
{
    char devName[9];
    uint8_t eui_numbers;
    GenericEui_t devEui,dev_eeprom;
    uint8_t js=0,found=0;
ESP_LOGI(TAG,"before n_of_eui=%d\n",get_eui_numbers());
    strcpy(devName,"DEV0EUI");
    for(uint8_t j=1;j<=7;j++)
    {
        devName[3]=0x30 + j;
        set_s(devName,&devEui);
        ESP_LOGI(TAG,"j=%d Eui=0x%016llX\n",j,devEui.eui);
        if(euicmpnz(&devEui))
        {
//            send_chars("not zero\r\n");
            found=0;
            eui_numbers=get_eui_numbers();
            for(uint8_t k=0;k<eui_numbers;k++)
            {
                if((get_Eui(k,&(dev_eeprom)))==ESP_OK)
                {
                     if(!euicmp(&devEui,&(dev_eeprom)) && !get_EUI_type(k))
                     {
                         found=1;
                         ESP_LOGI(TAG,"found j=%d Eui=0x%016llX k=%d\n",j,devEui.eui,k);
                         break;
                     }
                }
                else return 0;
            }
            if(!found)
            {
                put_Eui(eui_numbers,&devEui);
                put_DevNonce(eui_numbers,0);
                clear_EUI_type(eui_numbers);
                eui_numbers=increase_eui_numbers();
                Commit_deveui();
                ESP_LOGI(TAG,"not found j=%d Eui=0x%016llX k=%d\n",j,devEui.eui,get_eui_numbers()-1);
            }
        }
    }
    js=0;
//    printVar("after EEPROM_types=",PAR_UI32,&EEPROM_types,true,true);
    for(uint8_t j=0;j<eui_numbers;j++)
    {
        if(get_Eui(j,&devices[js].Eui)==ESP_OK && !get_EUI_type(j))
        {
            devices[js].DevNonce=get_DevNonce(j);
            ESP_LOGI(TAG,"Device number=%d DevNonce=0x%04X EUI=0x%016llX\n",js,devices[js].DevNonce,devices[js].Eui.eui);
            devices[js].js=j;
            js++;
        }
//        else
//       {
//           printVar(" not good j=",PAR_UI8,&j,false,false);
//           printVar(" Eui=",PAR_EUI64,&(devices[js].Eui),true,true);
//        }
    }

    return js;
}



