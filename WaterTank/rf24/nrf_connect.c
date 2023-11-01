#include "nrf_connect.h"

void NRF_connect_init() {
    PORTC &= ~(1 << PINC2);
    PORTB |= (1 << PINB2);

    DDRB |= (1 << PINB1) | (1 << PINB2) | (1 << PINB3) | (0 << PINB4) | (1 << PINB5);
    DDRC |= (1 << PINC2);

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

uint8_t SPI_0_exchange_byte(uint8_t data)
{
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

void SPI_0_exchange_block(void *block, uint8_t size) {
	uint8_t *b = (uint8_t *)block;
	while (size--) {
		SPDR = *b;
		while (!(SPSR & (1 << SPIF)));
		*b = SPDR;
		b++;
	}
}

void SPI_0_write_block(void *block, uint8_t size) {
	uint8_t *b = (uint8_t *)block;
	while (size--) {
		SPDR = *b;
		while (!(SPSR & (1 << SPIF)));
		b++;
	}
}

void SPI_0_read_block(void *block, uint8_t size) {
	uint8_t *b = (uint8_t *)block;
	while (size--) {
		SPDR = 0;
		while (!(SPSR & (1 << SPIF)));
		*b = SPDR;
		b++;
	}
}

void CSN_set_level(bool level) {
    if (level) {
        PORTB |= (1 << PINB2);
    } else {
        PORTB &= ~(1 << PINB2);
    }     
}

void CE_set_level(bool level) {
    if (level) {
        PORTC |= (1 << PINC2);
        } else {
        PORTC &= ~(1 << PINC2);
    }
}