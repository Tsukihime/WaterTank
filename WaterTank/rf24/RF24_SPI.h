#ifndef RF24_SPI_H_
#define RF24_SPI_H_

#include <avr/io.h>

void RF24_spi_init(void);

uint8_t RF24_spi_exchange_byte(uint8_t data);

void RF24_spi_write_block(void *block, uint8_t size);

void RF24_spi_read_block(void *block, uint8_t size);

void RF24_csn_set(uint8_t level);

void RF24_ce_set(uint8_t level);

#endif /* RF24_SPI_H_ */
