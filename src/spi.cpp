#include <Arduino.h>
#include <SPI.h>
#include "config.h"
#include <esp_system.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

spi_device_handle_t spiImu;
spi_device_handle_t spiBaro;

static spi_bus_config_t buscfg;
static spi_device_interface_config_t imudevcfg;
static spi_device_interface_config_t barodevcfg;


void spi_init() {
	esp_err_t ret;
	buscfg = {};
	buscfg.miso_io_num = pinMISO;
	buscfg.mosi_io_num = pinMOSI;
	buscfg.sclk_io_num = pinSCK;
	buscfg.quadwp_io_num = -1;
	buscfg.quadhd_io_num = -1;
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

	imudevcfg = {};
	imudevcfg.address_bits     = 0;
  	imudevcfg.command_bits     = 0;
  	imudevcfg.dummy_bits       = 0;
	imudevcfg.clock_speed_hz = 1000000;           //Clock out at 1 MHz
	imudevcfg.mode = SPI_MODE0;                                //SPI mode 0
	imudevcfg.duty_cycle_pos   = 0;
  	imudevcfg.cs_ena_posttrans = 0;
  	imudevcfg.cs_ena_pretrans  = 0;
	imudevcfg.spics_io_num = pinNCS;                    //CS pin
	imudevcfg.queue_size = 1;          //We want to be able to queue 7 transactions at a time
	imudevcfg.pre_cb = NULL;  
	imudevcfg.post_cb = NULL;  

    ret = spi_bus_add_device(SPI2_HOST, &imudevcfg, &spiImu);
    ESP_ERROR_CHECK(ret);

	barodevcfg = {};
	barodevcfg.address_bits     = 0;
  	barodevcfg.command_bits     = 0;
  	barodevcfg.dummy_bits       = 0;
	barodevcfg.clock_speed_hz = 1000000;           //Clock out at 1 MHz
	barodevcfg.mode = SPI_MODE0;                                //SPI mode 0
	barodevcfg.duty_cycle_pos   = 0;
  	barodevcfg.cs_ena_posttrans = 0;
  	barodevcfg.cs_ena_pretrans  = 0;
	barodevcfg.spics_io_num = pinCSB;                    //CS pin
	barodevcfg.queue_size = 1;         
	barodevcfg.pre_cb = NULL;  
	barodevcfg.post_cb = NULL;  

    ret = spi_bus_add_device(SPI2_HOST, &barodevcfg, &spiBaro);
    ESP_ERROR_CHECK(ret);
	}


void spi_write_command(spi_device_handle_t dev, uint8_t cmd){
	spi_transaction_t tdesc = {};
	tdesc.length = 8; // num xfer bits
	tdesc.tx_buffer = &cmd;
	esp_err_t ret = spi_device_polling_transmit(dev, &tdesc);
    ESP_ERROR_CHECK(ret);
	} 

void spi_write_register(spi_device_handle_t dev, uint8_t addr, uint8_t data){
	uint8_t txdata[2] = {};
	txdata[0] = addr;
	txdata[1] = data;
	spi_transaction_t tdesc = {};
	tdesc.length = 8 * 2; // num xfer bits
	tdesc.tx_buffer = txdata;
	esp_err_t ret = spi_device_polling_transmit(dev, &tdesc);
    ESP_ERROR_CHECK(ret);
	} 


uint8_t spi_read_register(spi_device_handle_t dev, uint8_t addr){
	uint8_t rxdata[2] = {};
	uint8_t txdata[2] = {};
	txdata[0] = addr;
	spi_transaction_t tdesc = {};
	tdesc.length = 8 * 2;  // num xfer bits
	tdesc.tx_buffer = txdata;
	tdesc.rxlength = 8 *2; 
	tdesc.rx_buffer = rxdata;
	esp_err_t ret = spi_device_polling_transmit(dev, &tdesc);
    ESP_ERROR_CHECK(ret);
	return rxdata[1];
	}	 

#define MAX_XFER_BYTES 21

void spi_read_buffer(spi_device_handle_t dev, uint8_t addr, int numBytes, uint8_t* pbuf){
	uint8_t rxdata[MAX_XFER_BYTES] = {};
	uint8_t txdata[MAX_XFER_BYTES] = {};
	txdata[0] = addr;
	spi_transaction_t tdesc = {};
	tdesc.length = 8 * (numBytes+1);  // num xfer bits
	tdesc.tx_buffer = txdata;
	tdesc.rxlength = 8 * (numBytes+1); 
	tdesc.rx_buffer = rxdata;
	esp_err_t ret = spi_device_polling_transmit(dev, &tdesc);
    ESP_ERROR_CHECK(ret);
	memcpy(pbuf, &rxdata[1], numBytes);
	}	 

