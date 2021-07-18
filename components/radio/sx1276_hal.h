/*
 * sx1276_hal.h
 *
 *  Created on: 18 июл. 2021 г.
 *      Author: ilya_000
 */

#ifndef COMPONENTS_RADIO_SX1276_HAL_H_
#define COMPONENTS_RADIO_SX1276_HAL_H_

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
//													func1	func2
#define PIN_NUM_SX1276_MISO		19	// IO19 		GPIO19	VSPIQ
#define PIN_NUM_SX1276_MOSI		23 	// IO23			GPIO23	VSPID
#define PIN_NUM_SX1276_CLK		18	// IO18			GPIO18	VSPICLK
#define PIN_NUM_SX1276_CSN		5	// IO5			GPIO5	VSPICS0
#define PIN_NUM_SX1276_NRESET	22	// IO22 		GPIO22	VSPIWP
#define PIN_NUM_SX1276_DIO0		39	// SENSOR_VN	GPIO39			INPUT_ONLY
#define PIN_NUM_SX1276_DIO1		38 	// SENSOR_CAPN	GPIO38			INPUT_ONLY
#define PIN_NUM_SX1276_DIO2		37	// SENSOR_CAPP	GPIO37			INPUT_ONLY
#define PIN_NUM_SX1276_DIO3		36	// SENSOR_VP	GPIO36			INPUT_ONLY
#define PIN_NUM_SX1276_DIO4		35	// IO35 VDET_2	GPIO35			INPUT_ONLY
#define PIN_NUM_SX1276_DIO5		34	// IO34 VDET_1	GPIO34			INPUT_ONLY
#define PIN_NUM_SX1276_V1		17	// IO17 		GPIO17

#define MY_SPI_HOST				SPI3_HOST

#define DIO0        0x01
#define DIO1        0x02
#define DIO2        0x04
#define DIO3        0x08
#define DIO4        0x10
#define DIO5        0x20

void HALResetPinMakeOutput(void);
void HALResetPinMakeInput(void);
void HALResetPinOutputValue(uint8_t value);
void V1_SetLow(void);
void V1_SetHigh(void);


void HALSPICSAssert(void);
void HALSPICSDeassert(void);
uint8_t HALSPISend(uint8_t data);
void RADIO_RegisterWrite(uint8_t reg, uint8_t value);
uint8_t RADIO_RegisterRead(uint8_t reg);
void HALSPIWriteFIFO(uint8_t* data, uint8_t datalen);
void HALSPIReadFIFO(uint8_t* data, uint8_t datalen);
void HALSPIReadFSKFIFO(uint8_t* data, uint8_t* datalen);
void DIO_ISR_Lora(void* DIO);
void HALDioInterruptInit(void);

uint8_t HALDIO0PinValue(void);
uint8_t HALDIO1PinValue(void);
uint8_t HALDIO2PinValue(void);
uint8_t HALDIO5PinValue(void);



#ifdef	__cplusplus
}
#endif

#endif /* COMPONENTS_RADIO_SX1276_HAL_H_ */
