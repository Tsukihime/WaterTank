#include "RF24_SPI.h"

void RF24_spi_init() {
	RF24_ce_set(0);
	RF24_csn_set(1);

    DDRB |= (1 << PINB1) | (1 << PINB2) | (1 << PINB3) | (1 << PINB5);
	DDRB &= ~(1 << PINB4);

	PRR &= ~(1 << PRSPI); // Enable SPI
    SPSR = (1 << SPI2X);  // Double SPI speed

	SPCR = 1 << SPE                     /* SPI module enable: enabled */
	       | 0 << DORD                  /* Data order: disabled */
	       | 1 << MSTR                  /* Master/Slave select: enabled */
	       | 0 << CPOL                  /* Clock polarity: disabled */
	       | 0 << CPHA                  /* Clock phase: disabled */
	       | 0 << SPIE                  /* SPI interrupt enable: disabled */
	       | (0 << SPR1) | (0 << SPR0); /* SPI Clock rate selection: fosc/4 */
}

uint8_t RF24_spi_exchange_byte(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

void RF24_spi_write_block(void *block, uint8_t size) {
	uint8_t *b = (uint8_t *)block;
	while (size--) {
		SPDR = *b;
		while (!(SPSR & (1 << SPIF)));
		b++;
	}
}

void RF24_spi_read_block(void *block, uint8_t size) {
	uint8_t *b = (uint8_t *)block;
	while (size--) {
		SPDR = 0;
		while (!(SPSR & (1 << SPIF)));
		*b = SPDR;
		b++;
	}
}

void RF24_csn_set(uint8_t level) {
    if (level) {
        PORTB |= (1 << PINB2);
    } else {
        PORTB &= ~(1 << PINB2);
    }     
}

void RF24_ce_set(uint8_t level) {
    if (level) {
        PORTB |= (1 << PINB1);
    } else {
        PORTB &= ~(1 << PINB1);
    }
}
