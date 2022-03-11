#ifndef SPI_H_
#define SPI_H_

#include <driver/spi_master.h>

extern spi_device_handle_t spiImu;
extern spi_device_handle_t spiBaro;


void spi_init();
void spi_write_command(spi_device_handle_t dev, uint8_t cmd);
void spi_write_register(spi_device_handle_t dev, uint8_t addr, uint8_t data);
uint8_t spi_read_register(spi_device_handle_t dev, uint8_t addr);
void spi_read_buffer(spi_device_handle_t dev, uint8_t addr, int numBytes, uint8_t* pbuf);



#endif