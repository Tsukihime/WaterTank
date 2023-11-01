/*
 * RF24.c
 *	
 * Adapted from TMRh20's RF24 library
 * 
 * Created: 7/3/2019 1:53:45 AM
 *  Author: MatthewTran
 */ 

#include <util/delay.h>

#include "nRF24L01.h"
#include "RF24.h"
#include "nrf_connect.h"

uint8_t payload_size = 32;
uint8_t addr_width = 5;
bool dynamic_payloads_enabled;
static const uint8_t child_pipe_enable[] = { ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5 };
static const uint8_t child_pipe[] = { RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5 };
static const uint8_t child_payload_size[] = { RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5 };

// register writing/reading extractions
uint8_t RF24_read_register(uint8_t reg, uint8_t* buf, uint8_t len) {
	uint8_t status;
	CSN_set_level(false);
	status = SPI_0_exchange_byte(R_REGISTER | (REGISTER_MASK & reg));
	SPI_0_read_block(buf, len);
	CSN_set_level(true);
	return status;
}

uint8_t RF24_read_one_register(uint8_t reg) {
	uint8_t val = 0;
	CSN_set_level(false);
	SPI_0_exchange_byte(R_REGISTER | (REGISTER_MASK & reg));
	val = SPI_0_exchange_byte(0xFF);
	CSN_set_level(true);
	return val;
}

uint8_t RF24_write_register(uint8_t reg, uint8_t* buf, uint8_t len) {
	uint8_t status;
	CSN_set_level(false);
	status = SPI_0_exchange_byte(W_REGISTER | (REGISTER_MASK & reg));
	SPI_0_write_block(buf, len);
	CSN_set_level(true);
	return status;
}

uint8_t RF24_write_one_register(uint8_t reg, uint8_t val) {
	uint8_t status;
	CSN_set_level(false);
	status = SPI_0_exchange_byte(W_REGISTER | (REGISTER_MASK & reg));
	SPI_0_exchange_byte(val);
	CSN_set_level(true);
	return status;
}

//payload stuff
uint8_t RF24_write_payload(void* buf, uint8_t data_len, uint8_t writeType) {
	uint8_t status;
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
	CSN_set_level(false);
	status = SPI_0_exchange_byte(writeType);
	SPI_0_write_block(buf, data_len);
	while (blank_len--) {
		SPI_0_exchange_byte(0);
	}
	CSN_set_level(true);
	return status;
}

uint8_t RF24_read_payload(void* buf, uint8_t data_len) {
	uint8_t status;
	if (data_len > payload_size) {
		data_len = payload_size;
	}
	uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
	CSN_set_level(false);
	status = SPI_0_exchange_byte(R_RX_PAYLOAD);
	SPI_0_read_block(buf, data_len);
	while(blank_len--) {
		SPI_0_exchange_byte(0xFF);
	}
	CSN_set_level(true);
	return status;
}

//stuff for begin()
void RF24_flush_rx(void) {
	CSN_set_level(false);
	SPI_0_exchange_byte(FLUSH_RX);
	CSN_set_level(true);
}

void RF24_flush_tx(void) {
	CSN_set_level(false);
	SPI_0_exchange_byte(FLUSH_TX);
	CSN_set_level(true);
}

void RF24_toggle_features(void) { //idk what this does
	CSN_set_level(false);
	SPI_0_exchange_byte(ACTIVATE);
	SPI_0_exchange_byte(0x73);
	CSN_set_level(true);
}

void RF24_set_retries(uint8_t delay, uint8_t count) {
	RF24_write_one_register(SETUP_RETR, (delay & 0xF) << ARD | (count & 0xF) << ARC);
}

void RF24_powerUp(void) {
	uint8_t cfg = RF24_read_one_register(NRF_CONFIG);
	if (!(cfg & (1 << PWR_UP))) {
		RF24_write_one_register(NRF_CONFIG, cfg | (1 << PWR_UP));
		_delay_ms(5);
	}
}

void RF24_powerDown(void) {
	CE_set_level(false);
	RF24_write_one_register(NRF_CONFIG, RF24_read_one_register(NRF_CONFIG) & ~(1 << PWR_UP));
}

//lib stuff
void RF24_setChannel(uint8_t ch) {
	RF24_write_one_register(RF_CH, rf24_min(ch, 125));
}

void RF24_setPayloadSize(uint8_t size) {
	payload_size = rf24_min(size, 32);
}

void RF24_setDataRate(uint8_t rate) {
	//0 = 250kbps, 1 = 1mbps, 2 = 2mbps
	uint8_t setup = RF24_read_one_register(RF_SETUP);
	setup &= ~(1 << RF_DR_LOW | 1 << RF_DR_HIGH); //reset to 1mbps	
	if (rate == 0) {
		setup |= 1 << RF_DR_LOW;
	} else if (rate == 2) {
		setup |= 1 << RF_DR_HIGH;
	}
	RF24_write_one_register(RF_SETUP, setup);
}

void RF24_setPALevel(uint8_t level) {
	//0 = min, 3 = max
	uint8_t setup = RF24_read_one_register(RF_SETUP) & 0xF8; //clear level bits
	if (level > 3) {
		level = 3;
	}
	level = (level << 1); // + 1;
	RF24_write_one_register(RF_SETUP, setup |= level);
}

void RF24_enableDynamicPayloads(void) {
	RF24_write_one_register(FEATURE, RF24_read_one_register(FEATURE) | (1 << EN_DPL));
	RF24_write_one_register(DYNPD, 0x3F); //enable on all pipes
	dynamic_payloads_enabled = true;
}

void RF24_setCRCLength(uint8_t length) {
    // 0 = OFF, 1 = 8bit, 2 = 16bit
    uint8_t cfg = RF24_read_one_register(NRF_CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));

    if (length == 0) {
        // Do nothing, we turned it off above.
    }
    else if (length == 1) {
        cfg |= 1 << EN_CRC;
    }
    else {
        cfg |= 1 << EN_CRC;
        cfg |= 1 << CRCO;
    }

    RF24_write_one_register(NRF_CONFIG, cfg);
}

void RF24_setAutoAck(bool enable) {
    if (enable) {
        RF24_write_one_register(EN_AA, 0x3f);
    } else {
        RF24_write_one_register(EN_AA, 0);
    }
}

void RF24_begin(void) {
	//_delay_ms(5); //radio settle
	RF24_write_one_register(NRF_CONFIG, 0x0C); //reset, enable 16-bit CRC
	RF24_set_retries(5, 15); //1500us, 15 retries
	RF24_setDataRate(1);
	RF24_toggle_features();
	RF24_write_one_register(FEATURE, 0);
	RF24_write_one_register(DYNPD, 0);
	dynamic_payloads_enabled = false;
	RF24_write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT);
	RF24_flush_rx();
	RF24_flush_tx();
	RF24_powerUp();
	RF24_write_one_register(NRF_CONFIG, RF24_read_one_register(NRF_CONFIG) & ~(1 << PRIM_RX));
}

void RF24_openWritingPipe(uint8_t* address) {
	RF24_write_register(RX_ADDR_P0, address, addr_width); //TX_ADDR must match RX_ADDR_P0
	RF24_write_register(TX_ADDR, address, addr_width);
	RF24_write_one_register(RX_PW_P0, payload_size);
}

void RF24_openReadingPipe(uint8_t child, uint8_t* address) {
	//not gonna support setting child 0, just use openWritingPipe()
	if (child == 0 || child > 5) { return; }
	if ( child < 2) {
		RF24_write_register(child_pipe[child], address, addr_width);
	} else {
		RF24_write_register(child_pipe[child], address, 1);
	}
	RF24_write_one_register(child_payload_size[child], payload_size);
	RF24_write_one_register(EN_RXADDR, RF24_read_one_register(EN_RXADDR) | 1 << child_pipe_enable[child]);
}

void RF24_startListening(void) {
	RF24_write_one_register(NRF_CONFIG, RF24_read_one_register(NRF_CONFIG) | 1 << PRIM_RX); //enable RX
	RF24_write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); //clear interrupts
	CE_set_level(true);
	//pipe 0 always open
	
	//not implemented, but moved code over anyway
	//for if want to send ack payload
	if (RF24_read_one_register(FEATURE) & 1 << EN_ACK_PAY) {
		RF24_flush_tx();
	}
}

void RF24_stopListening(void) {
	CE_set_level(false);
	_delay_us(200); //didn't implement txDelay, so just using a num
	
	//again didn't implement, but here just because
	if (RF24_read_one_register(FEATURE) & 1 << EN_ACK_PAY) {
		_delay_us(200);
		RF24_flush_tx();
	}
	
	//back to tx
	RF24_write_one_register(NRF_CONFIG, RF24_read_one_register(NRF_CONFIG) & ~(1 << PRIM_RX));
	//enable rx on pipe0 again, probs bc of the ack, unnecessary for current implementation
	RF24_write_one_register(EN_RXADDR, RF24_read_one_register(EN_RXADDR) | 1 << ERX_P0);
}

uint8_t get_status() {
    CSN_set_level(false);
    uint8_t status = SPI_0_exchange_byte(RF24_NOP);
    CSN_set_level(true);
    return status;
}

void RF24_write(void* buf, uint8_t len, bool multicast) {
	//cant do multicast == true w/o enabling dynamic ack (check RF24)
	RF24_write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    _delay_us(10);
	CE_set_level(true);
	_delay_us(10);
	CE_set_level(false);

    while (!(get_status() & ((1 << TX_DS) | (1 << MAX_RT)))) {
        _delay_ms(1);
    }

    uint8_t status = RF24_write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT);
    if (status & (1 << MAX_RT)) {
        RF24_flush_tx(); // Only going to be 1 packet in the FIFO at a time using this method, so just flush
    }
}

void RF24_read(void* buf, uint8_t len) {
	RF24_read_payload(buf, len);
	//using interrupts, shouldn't need this if have proper ISR
	RF24_write_one_register(NRF_STATUS, 1 << RX_DR | 1 << MAX_RT | 1 << TX_DS);
}

void RF24_whatHappened(uint8_t* tx_ok, uint8_t* tx_fail, uint8_t* rx_ready) {
	uint8_t status = RF24_write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT);
	*tx_ok = status & 1 << TX_DS;
	*tx_fail = status & 1 << MAX_RT;
	*rx_ready = status & 1 << RX_DR;
}
