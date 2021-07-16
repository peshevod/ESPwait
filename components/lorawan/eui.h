/*
 * eui.h
 *
 *  Created on: 16 июл. 2021 г.
 *      Author: ilya_000
 */



#ifndef COMPONENTS_LORAWAN_EUI_H_
#define COMPONENTS_LORAWAN_EUI_H_

#define	TYPE_DEVEUI		0
#define	TYPE_JOINEUI	1

typedef union
{
    uint8_t buffer[8];
    uint64_t eui;
    struct
    {
        uint32_t genericEuiL;
        uint32_t genericEuiH;
    }members;
} GenericEui_t;

uint8_t fill_devices(void);
uint8_t selectJoinServer(void* joinServer);

#endif /* COMPONENTS_LORAWAN_EUI_H_ */
