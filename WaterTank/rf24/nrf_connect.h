#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <stdbool.h>

void NRF_connect_init(void);

uint8_t SPI_0_exchange_byte(uint8_t data);

void SPI_0_exchange_block(void *block, uint8_t size);

void SPI_0_write_block(void *block, uint8_t size);

void SPI_0_read_block(void *block, uint8_t size);

void CSN_set_level(bool level);

void CE_set_level(bool level);

#endif /* SPI_H_ */
