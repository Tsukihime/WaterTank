/*
 * RF24.h
 *
 * Adapted from TRMh20's RF24 library
 *
 * Created: 7/3/2019 1:19:34 AM
 *  Author: MatthewTran
 */ 


#ifndef RF24_H_
#define RF24_H_

#include <stdbool.h>
#include <stdint.h>

#define RF24_PA_MIN  0
#define RF24_PA_LOW  1
#define RF24_PA_HIGH 2
#define RF24_PA_MAX  3

#define RF24_250KBPS 0
#define RF24_1MBPS   1
#define RF24_2MBPS   2

#define RF24_CRC_DISABLED 0
#define RF24_CRC_8        1
#define RF24_CRC_16       2

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

void RF24_begin(void);

void RF24_setChannel(uint8_t ch);

void RF24_setPayloadSize(uint8_t size);

void RF24_setPALevel(uint8_t level);

void RF24_setDataRate(uint8_t rate);

void RF24_enableDynamicPayloads(void);

void RF24_openWritingPipe(uint8_t* address);

void RF24_openReadingPipe(uint8_t child, uint8_t* address);

void RF24_startListening(void);

void RF24_stopListening(void);

void RF24_write(void* buf, uint8_t len, bool multicast);

void RF24_read(void* buf, uint8_t len);

void RF24_whatHappened(uint8_t* tx_ok, uint8_t* tx_fail, uint8_t* rx_ready);

void RF24_powerUp(void);

void RF24_powerDown(void);

void RF24_setCRCLength(uint8_t length);

void RF24_setAutoAck(bool ack);

uint8_t RF24_read_register(uint8_t reg, uint8_t* buf, uint8_t len);

#endif /* RF24_H_ */
