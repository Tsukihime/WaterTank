/*
 * RF24.h
 *
 * Adapted from TRMh20's RF24 library
 *
 * Created: 7/3/2019 1:19:34 AM
 *  Author: MatthewTran
 *  Modified: Tsukihime
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

class RF24 {
public:
    typedef uint8_t (*SPIExchangeFunc)(uint8_t);
    typedef void (*PinSetFunc)(bool, bool);

    RF24(SPIExchangeFunc spi_func, PinSetFunc pin_set_func)
    : spi_exchange_byte(spi_func), rf_pin_set(pin_set_func) {}
        
    void begin();
    void setChannel(uint8_t ch);
    void setPayloadSize(uint8_t size);
    void setPALevel(uint8_t level);
    void setDataRate(uint8_t rate);
    void enableDynamicPayloads();
    void openWritingPipe(uint8_t* address);
    void openReadingPipe(uint8_t child, uint8_t* address);
    void startListening();
    void stopListening();
    bool write(uint8_t* buf, uint8_t len, bool multicast);
    void read(uint8_t* buf, uint8_t len);
    void whatHappened(uint8_t* tx_ok, uint8_t* tx_fail, uint8_t* rx_ready);
    void powerUp();
    void powerDown();
    void setCRCLength(uint8_t length);
    void setAutoAck(bool ack);

private:
    uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t read_one_register(uint8_t reg);
    uint8_t write_register(uint8_t reg, uint8_t* buf, uint8_t len);
    uint8_t write_one_register(uint8_t reg, uint8_t val);
    uint8_t write_payload(uint8_t* buf, uint8_t data_len, uint8_t writeType);
    uint8_t read_payload(uint8_t* buf, uint8_t data_len);
    void set_retries(uint8_t delay, uint8_t count);
    void toggle_features();
    void flush_rx();
    void flush_tx();
    uint8_t get_status();
    
    void set_pins();
    
    bool CSN_pin;
    bool CE_pin;
    uint8_t payload_size;
    bool dynamic_payloads_enabled;
    SPIExchangeFunc spi_exchange_byte;
    PinSetFunc rf_pin_set;
};

#endif /* RF24_H_ */
