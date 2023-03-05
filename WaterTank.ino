#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

//#define DEBUG_SERIAL 1

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(9, 10);

const uint8_t address[6] = { "NrfMQ" };
const uint8_t channel = 0x6f;

void initRadio() {
    radio.begin();                   // инициализация
    radio.setPALevel(RF24_PA_MAX);   // уровень питания усилителя RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
    radio.enableDynamicPayloads();   // динамический размер посылки
    radio.setDataRate(RF24_1MBPS);   // скорость обмена данными RF24_1MBPS или RF24_2MBPS
    radio.setCRCLength(RF24_CRC_8);  // размер контрольной суммы 8 bit или 16 bit
    radio.setChannel(channel);       // установка канала
    radio.setAutoAck(false);         // автоответ
    radio.openWritingPipe(address);  // открыть трубу на отправку
    radio.stopListening();           // радиоэфир не слушаем, только передача

#ifdef DEBUG_SERIAL
    radio.printDetails();
#endif
}

void initWatchdog() {
    MCUSR &= ~(1 << WDRF);              // Just to be safe since we can not clear WDE if WDRF is set
    cli();                              // disable interrupts so we do not get interrupted while doing timed sequence
    WDTCSR |= (1 << WDCE) | (1 << WDE); // First step of timed sequence, we have 4 cycles after this to make changes to WDE and WD timeout
//    WDTCSR = 1 << WDP0 | 1 << WDP2;     // timeout in 0.5 second, disable reset mode. Must be done in one operation
    WDTCSR = 1 << WDP3 | 1 << WDP0;     // timeout in 8 second, disable reset mode. Must be done in one operation
    sei();
    WDTCSR |= _BV(WDIE);                // enable watchdog interrupt only mode 
}

ISR(WDT_vect) {}

void enableAdc() {
    ADMUX = (1 << REFS1) | (1 << REFS0); // Internal 1.1V Voltage Reference with external capacitor at AREF pin
    ADCSRA = (1 << ADEN)
           | (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Prescaler Selections Div8
    ADCSRA |= (1 << ADSC);                               // start dummy conversion
    while(bit_is_set(ADCSRA, ADSC));                     // wait for dummy to finish
}

void disableAdc() {
    ADCSRA &= ~(1 << ADEN);
}

uint16_t getBatteryVoltage() {
    ADMUX = (0 << REFS1) | (1 << REFS0) |                          // select AVCC as reference
            (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // measure bandgap reference voltage
    _delay_us(500);                  // a delay rather than a dummy measurement is needed to give a stable reading!
    ADCSRA |= (1 << ADSC);           // start conversion
    while(bit_is_set(ADCSRA, ADSC)); // wait to finish
    return (1080UL * 1023 / ADC);    // AVcc = Vbg / ADC * 1023 = 1.1V * 1023 / ADC
}

int8_t getInternalTemperature() {
    ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3); // Read temperature sensor against 1.1V reference
    _delay_us(1500);
    ADCSRA |= (1 << ADSC);           // Start AD conversion
    while(bit_is_set(ADCSRA, ADSC)); // Detect end-of-conversion
    return (ADC * 100 - 31063) / 118;
}

uint8_t getWatertankLevel() {
    const uint8_t PORD_MASK = 0b11111100;
    const uint8_t PORC_MASK = 0b00000011;

    // enable pullup
    PORTD |= PORD_MASK;
    PORTC |= PORC_MASK;
    _delay_ms(1); // recharge parasite capacitance

    uint8_t electrodes = ~(PIND & PORD_MASK | PINC & PORC_MASK);

    // disable pullup
    PORTD &= ~PORD_MASK;
    PORTC &= ~PORC_MASK;

    uint8_t level = 0;
    for(int i = 0; i < 8; i++) {
        if(electrodes & 1) {
            level++;
        } else {
            break;
        }
        electrodes >>= 1;
    }
    level *= 15; // 0..8 => 0..120
    return level;
}

void enterSleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
}

void sendMQTTMessage(const char* topic, const char* payload) {
    char buffer[32];
    uint8_t topicLength = strlen(topic);
    uint8_t payloadLength = strlen(payload);
    uint8_t messageLength = topicLength + payloadLength + 2;
    if(messageLength > 32) return;

    buffer[0] = topicLength;
    buffer[1] = payloadLength;
    memcpy(&buffer[2], topic, topicLength);
    memcpy(&buffer[2 + topicLength], payload, payloadLength);
    radio.write(buffer, messageLength);
}

char* uintToStr(uint16_t value, char buffer[5]) {
    uint32_t subtrahend = 10000;
    for (uint8_t i = 0; i < 5; i++) {
        buffer[i] = '0';
        while (value >= subtrahend) {
            value -= subtrahend;
            buffer[i]++;
        }
        subtrahend /= 10;
    }
    for(uint8_t i = 0; i < 5; i++) {
        if(buffer[i] != '0') return &buffer[i];
    }
    return &buffer[4];
}

void sendWatertankLevel(uint8_t level) {
    char buff[5 + 1];
    char* payload = uintToStr(level, buff);
    buff[5] = '\0';
    sendMQTTMessage("watertank/level", payload);
}

void sendBatteryVoltage(uint16_t voltage) {
    char payload[5];
    uintToStr(voltage, payload);
    payload[0] = payload[1];
    payload[1] = '.';
    payload[4] = '\0';
    sendMQTTMessage("watertank/batt", payload);
}

void sendInternalTemperature(int8_t temperature) {
    char buff[5 + 2];
    bool negative = temperature < 0;
    uint16_t temp = (negative) ? -temperature : temperature;

    char* payload = uintToStr(temp, &buff[1]);
    buff[6] = '\0';

    if(negative) {
        payload--;
        payload[0] = '-';
    }
    sendMQTTMessage("watertank/temp", payload);
}

void setup() {
    // all input
    DDRB &= 0b00111110; // pb1..pb5 dont touch nrf24
    DDRC = 0;
    DDRD &= 0b00000011; // dont touch uart

    initRadio();
    initWatchdog();
    sendMQTTMessage("watertank", "Start!");

#ifdef DEBUG_SERIAL
    Serial.begin(9600);
    Serial.println("Reboot!");
#endif
}

void loop() {
    const uint8_t period = 225; // 8sec * 225 = update every 30 min or on level change every 8 sec
    static uint8_t counter = 0;
    uint8_t level = getWatertankLevel();
    static uint8_t old_level;

#ifdef DEBUG_SERIAL
    Serial.println(level);
#endif

    if(level != old_level || counter == 0) {
        enableAdc();
        uint16_t voltage = getBatteryVoltage();
        int8_t temp = getInternalTemperature();
        disableAdc();
      
        radio.powerUp();
        sendBatteryVoltage(voltage);
        sendWatertankLevel(level);
        sendInternalTemperature(temp);
        radio.powerDown();
    }
    old_level = level;
    
    if(++counter >= period) counter = 0;

#ifdef DEBUG_SERIAL
    delay(10);
#endif

    enterSleep();
}
