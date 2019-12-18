/*
 * spi_intf.c
 *
 *  Created on: 7 дек. 2019 г.
 *      Author: ilya_000
 */

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "spi_intf.h"

spi_device_handle_t spi;
esp_err_t ret;

spi_bus_config_t buscfg={
    .miso_io_num=PIN_NUM_MISO,
    .mosi_io_num=PIN_NUM_MOSI,
    .sclk_io_num=PIN_NUM_CLK,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=64
};

void spi_pre_transfer_callback(spi_transaction_t *t)
{
//	gpio_set_level(PIN_NUM_CS, 0);
}

void spi_post_transfer_callback(spi_transaction_t *t)
{
//	gpio_set_level(PIN_NUM_CS, 1);
}

spi_device_interface_config_t devcfg={
	.command_bits=8,
	.address_bits=8,
	.dummy_bits=0,
    .mode=1,	//SPI mode 1
	.duty_cycle_pos=0,
	.cs_ena_posttrans=0,
    .clock_speed_hz=5*1000*1000,           //Clock out at 10 MHz
	.input_delay_ns=0,
    .spics_io_num=PIN_NUM_CS,               //CS pin
    .spics_io_num=-1,               //CS pin
    .queue_size=7,                          //We want to be able to queue 7 transactions at a time
	.pre_cb=spi_pre_transfer_callback,
	.post_cb=spi_post_transfer_callback
};

void init_spi_intf()
{

	gpio_set_direction(PIN_NUM_SDN, GPIO_MODE_OUTPUT);
	gpio_iomux_out(PIN_NUM_CS, FUNC_MTDO_HSPICS0, false);
    gpio_iomux_in(PIN_NUM_MISO, HSPIQ_IN_IDX);
    gpio_iomux_out(PIN_NUM_MOSI, FUNC_MTCK_HSPID, false);
    gpio_iomux_out(PIN_NUM_CLK, FUNC_MTMS_HSPICLK, false);
    gpio_set_level(PIN_NUM_SDN, 0);
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    printf("bus init ret=%d\n",ret);
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    printf("add device ret=%d\n",ret);
}

void get_status()
{
	spi_transaction_t t;
	uint8_t trans[64];
	uint8_t rec[64];
	t.flags=0;
	t.cmd=1;
	t.addr=0x8d;
	t.tx_buffer=trans;
	t.rx_buffer=rec;
	t.length=16;
	t.rxlength=0;
	trans[0]=1;
	trans[1]=0x8d;
	ret=spi_device_polling_transmit(spi, &t);
	ESP_ERROR_CHECK(ret);
	printf("%d %d %d %d %d\n",ret ,rec[0],rec[1],rec[2],rec[3]);
}
