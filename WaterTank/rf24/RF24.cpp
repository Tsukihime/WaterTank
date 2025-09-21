/*
 * RF24.c
 *    
 * Adapted from TMRh20's RF24 library
 * 
 * Created: 7/3/2019 1:53:45 AM
 *  Author: MatthewTran
 *  Modified: Tsukihime
 */ 

#include <util/delay.h>

#include "nRF24L01.h"
#include "RF24.h"

const uint8_t addr_width = 5;
static const uint8_t child_pipe_enable[] = { ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5 };
static const uint8_t child_pipe[] = { RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5 };
static const uint8_t child_payload_size[] = { RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5 };

#define CSN_HI()    do { CSN_pin = true; set_pins(); } while(0)
#define CSN_LOW()   do { CSN_pin = false; set_pins(); } while(0)
#define CE_HI()     do { CE_pin = true; set_pins(); } while(0)
#define CE_LOW()    do { CE_pin = false; set_pins(); } while(0)

void RF24::set_pins() {
    rf_pin_set(CSN_pin, CE_pin);
}

// register writing/reading extractions
uint8_t RF24::read_command(uint8_t command, uint8_t* buf, uint8_t len) {
    CSN_LOW();
    uint8_t status = spi_exchange_byte(command);
    while (len--) {
	    *buf++ = spi_exchange_byte(RF24_NOP);
    }
    CSN_HI();
    return status;
}

uint8_t RF24::write_command(uint8_t command, uint8_t* buf, uint8_t len) {
	CSN_LOW();
	uint8_t status = spi_exchange_byte(command);
	while (len--) {
		spi_exchange_byte(*buf++);
	}
	CSN_HI();
	return status;
}

uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len) {
    return read_command(R_REGISTER | (REGISTER_MASK & reg), buf, len);
}

uint8_t RF24::read_one_register(uint8_t reg) {
    uint8_t buf;
	read_command(R_REGISTER | (REGISTER_MASK & reg), &buf, 1);
    return buf;
}

uint8_t RF24::write_register(uint8_t reg, uint8_t* buf, uint8_t len) {
    return write_command(W_REGISTER | (REGISTER_MASK & reg), buf, len);
}

uint8_t RF24::write_one_register(uint8_t reg, uint8_t val) {
	return write_command(W_REGISTER | (REGISTER_MASK & reg), &val, 1);
}

//payload stuff
uint8_t RF24::write_payload(uint8_t* buf, uint8_t data_len, uint8_t writeType) {
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
    CSN_LOW();
    uint8_t status = spi_exchange_byte(writeType);
    while (data_len--) {
        spi_exchange_byte(*buf++);
    }
    while (blank_len--) {
        spi_exchange_byte(0);
    }
    CSN_HI();
    return status;
}

uint8_t RF24::read_payload(uint8_t* buf, uint8_t data_len) {
	data_len = rf24_min(data_len, payload_size);
    uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;
    CSN_LOW();
    uint8_t status = spi_exchange_byte(R_RX_PAYLOAD);
    while (data_len--) {
        *buf++ = spi_exchange_byte(RF24_NOP);
    }
        
    while (blank_len--) {
        spi_exchange_byte(RF24_NOP);
    }
    CSN_HI();
    return status;
}

//stuff for begin()
void RF24::flush_rx() {
	write_command(FLUSH_RX, 0, 0);
}

void RF24::flush_tx() {
	write_command(FLUSH_TX, 0, 0);
}

void RF24::toggle_features() { //idk what this does
	uint8_t data = 0x73;
	write_command(ACTIVATE, &data, 1);
}

void RF24::setRetries(uint8_t delay, uint8_t count) {
    write_one_register(SETUP_RETR, (delay & 0xF) << ARD | (count & 0xF) << ARC);
}

void RF24::powerUp() {
    uint8_t cfg = read_one_register(NRF_CONFIG);
    if (!(cfg & (1 << PWR_UP))) {
        write_one_register(NRF_CONFIG, cfg | (1 << PWR_UP));
        _delay_ms(5);
    }
}

void RF24::powerDown() {
    CE_LOW();
    write_one_register(NRF_CONFIG, read_one_register(NRF_CONFIG) & ~(1 << PWR_UP));
}

//lib stuff
void RF24::setChannel(uint8_t ch) {
    write_one_register(RF_CH, rf24_min(ch, 125));
}

void RF24::setPayloadSize(uint8_t size) {
    payload_size = rf24_min(size, 32);
}

void RF24::setDataRate(uint8_t rate) {
	uint8_t setup = read_one_register(RF_SETUP) & ~(1 << RF_DR_LOW | 1 << RF_DR_HIGH);  //reset to 1mbps
	if (rate == RF24_250KBPS) {
		setup |= (1 << RF_DR_LOW);
	} else if (rate == RF24_2MBPS) {
		setup |= (1 << RF_DR_HIGH);
	}
	write_one_register(RF_SETUP, setup);
}

void RF24::setPALevel(uint8_t level) {
    uint8_t setup = read_one_register(RF_SETUP) & 0xF8; //clear level bits
	write_one_register(RF_SETUP, setup | (rf24_min(level, RF24_PA_MAX) << 1));
}

void RF24::enableDynamicPayloads() {
    write_one_register(FEATURE, read_one_register(FEATURE) | (1 << EN_DPL));
    write_one_register(DYNPD, 0x3F); //enable on all pipes
    dynamic_payloads_enabled = true;
}

void RF24::setCRCLength(uint8_t length) {
	uint8_t cfg = read_one_register(NRF_CONFIG) & ~((1 << CRCO) | (1 << EN_CRC));
	switch (length) {
		case RF24_CRC_8:  cfg |= (1 << EN_CRC); break;
		case RF24_CRC_16: cfg |= (1 << EN_CRC) | (1 << CRCO); break;
		default: break; // RF24_CRC_DISABLED
	}
	write_one_register(NRF_CONFIG, cfg);
}

void RF24::setAutoAck(bool enable) {
    if (enable) {
        write_one_register(EN_AA, 0x3f);
    } else {
        write_one_register(EN_AA, 0);
    }
}

void RF24::begin() {
    CE_LOW();
    CSN_HI();
    //_delay_ms(5); //radio settle
    write_one_register(NRF_CONFIG, 0x0C); //reset, enable 16-bit CRC
    setRetries(5, 15); //1500us, 15 retries
    setDataRate(RF24_1MBPS);
    toggle_features();
    write_one_register(FEATURE, 0);
    write_one_register(DYNPD, 0);
    payload_size = 32;
    dynamic_payloads_enabled = false;
    write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT);
    flush_rx();
    flush_tx();
    powerUp();
    write_one_register(NRF_CONFIG, read_one_register(NRF_CONFIG) & ~(1 << PRIM_RX));
}

void RF24::openWritingPipe(uint8_t* address) {
    write_register(RX_ADDR_P0, address, addr_width); //TX_ADDR must match RX_ADDR_P0
    write_register(TX_ADDR, address, addr_width);
    write_one_register(RX_PW_P0, payload_size);
}

void RF24::openReadingPipe(uint8_t child, uint8_t* address) {
    //not gonna support setting child 0, just use openWritingPipe()
    if (child == 0 || child > 5) { return; }
    if ( child < 2) {
        write_register(child_pipe[child], address, addr_width);
    } else {
        write_register(child_pipe[child], address, 1);
    }
    write_one_register(child_payload_size[child], payload_size);
    write_one_register(EN_RXADDR, read_one_register(EN_RXADDR) | 1 << child_pipe_enable[child]);
}

void RF24::startListening() {
    write_one_register(NRF_CONFIG, read_one_register(NRF_CONFIG) | 1 << PRIM_RX); //enable RX
    write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT); //clear interrupts
    CE_HI();
    //pipe 0 always open
    
    //not implemented, but moved code over anyway
    //for if want to send ack payload
    if (read_one_register(FEATURE) & 1 << EN_ACK_PAY) {
        flush_tx();
    }
}

void RF24::stopListening() {
    CE_LOW();
    _delay_us(200); //didn't implement txDelay, so just using a num
    
    //again didn't implement, but here just because
    if (read_one_register(FEATURE) & 1 << EN_ACK_PAY) {
        _delay_us(200);
        flush_tx();
    }
    
    //back to tx
    write_one_register(NRF_CONFIG, read_one_register(NRF_CONFIG) & ~(1 << PRIM_RX));
    //enable rx on pipe0 again, probs bc of the ack, unnecessary for current implementation
    write_one_register(EN_RXADDR, read_one_register(EN_RXADDR) | 1 << ERX_P0);
}

uint8_t RF24::get_status() {
    CSN_LOW();
    uint8_t status = spi_exchange_byte(RF24_NOP);
    CSN_HI();
    return status;
}

bool RF24::write(void* buf, uint8_t len, bool multicast) {
    //cant do multicast == true w/o enabling dynamic ack (check RF24)
    write_payload((uint8_t*)buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    CE_HI();
    while (!(get_status() & ((1 << TX_DS) | (1 << MAX_RT)))); // wait while data transmitted
    CE_LOW();
    uint8_t status = write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT);
    if (status & (1 << MAX_RT)) {
        flush_tx(); // Only going to be 1 packet in the FIFO at a time using this method, so just flush
        return false;
    }
    return true;
}

void RF24::read(void* buf, uint8_t len) {
    read_payload((uint8_t*)buf, len);
    //using interrupts, shouldn't need this if have proper ISR
    write_one_register(NRF_STATUS, 1 << RX_DR | 1 << MAX_RT | 1 << TX_DS);
}

void RF24::whatHappened(uint8_t* tx_ok, uint8_t* tx_fail, uint8_t* rx_ready) {
    uint8_t status = write_one_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT);
    *tx_ok = status & 1 << TX_DS;
    *tx_fail = status & 1 << MAX_RT;
    *rx_ready = status & 1 << RX_DR;
}
