/*
 * spi_intf.h
 *
 *  Created on: 7 дек. 2019 г.
 *      Author: ilya_000
 */

#ifndef MAIN_SPI_INTF_H_
#define MAIN_SPI_INTF_H_

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15
#define PIN_NUM_SDN	 27

void init_spi_intf(void);
void get_status(void);

#endif /* MAIN_SPI_INTF_H_ */
