#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

#include "utils.h"

extern "C" {
    #include "rf24/nrf_connect.h"
    #include "rf24/RF24.h"
    #include "rf24/RF24MQTTGateway.h"
}

uint8_t gateway_address[6] = { "NrfMQ" };
const uint8_t gateway_channel = 0x6f;

void initWatchdog() {
    MCUSR &= ~(1 << WDRF);               // Just to be safe since we can not clear WDE if WDRF is set
    cli();                               // disable interrupts so we do not get interrupted while doing timed sequence
    wdt_reset();
    WDTCSR = (1 << WDCE) | (1 << WDE);   // First step of timed sequence, we have 4 cycles after this to make changes to WDE and WD timeout
    WDTCSR = (1 << WDP3) | (1 << WDP0) | // timeout in 8 second, disable reset mode,
             (1 << WDIE);                // enable watchdog interrupt only mode, must be done in one operation
    sei();
}

// WDT Interrupt Is Used To Wake Up CPU From Sleep Mode
EMPTY_INTERRUPT(WDT_vect);

void enterSleep(void) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
}

uint16_t getBatteryVoltage() {
    ADMUX = (0 << REFS1) | (1 << REFS0) |                  // select AVCC as reference
    (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0); // measure bandgap reference voltage

    ADCSRA = (1 << ADEN) |                      // enable ADC
    (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Prescaler Selections Div8

    _delay_us(500); // a delay rather than a dummy measurement is needed to give a stable reading!
    
    ADCSRA |= (1 << ADSC);                    // start conversion
    while(bit_is_set(ADCSRA, ADSC));          // wait to finish

    uint16_t voltage = (1059UL * 1024 / ADC); // AVcc = Vbg * 1024 / ADC
    ADCSRA &= ~(1 << ADEN);                   // disable ADC
    return voltage;                           // millivolts
}

int8_t getInternalTemperature() {
    ADMUX = (1 << REFS1) | (1 << REFS0) |                          // 1.1V reference
            (1 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0); // select Temperature Sensor

    ADCSRA = (1 << ADEN) |                      // enable ADC
    (0 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC Prescaler Selections Div8

    _delay_ms(2);

    ADCSRA |= (1 << ADSC);                   // Start AD conversion
    while(bit_is_set(ADCSRA, ADSC));         // Detect end-of-conversion

    int8_t temp = ADC - 317;
    ADCSRA &= ~(1 << ADEN);                  // disable ADC
    return temp;
}

uint8_t getWatertankLevel() {
    const uint8_t PORD_MASK = 0b11111100;
    const uint8_t PORB_MASK = 0b00000011;
    const uint8_t LEVELS[9] = {0, 10, 20, 30, 40, 50, 75, 100, 110};

    // enable pullup
    PORTD |= PORD_MASK;
    PORTB |= PORB_MASK;
    _delay_ms(1); // recharge parasite capacitance

    uint8_t electrodes = ~((PIND & PORD_MASK) >> 2 | (PINB & PORB_MASK) << 6);

    // disable pullup
    PORTD &= ~PORD_MASK;
    PORTB &= ~PORB_MASK;

    uint8_t level = 0;
    for(int i = 0; i < 8; i++) {
        if(electrodes & 1) {
            level++;
            } else {
            break;
        }
        electrodes >>= 1;
    }
    return LEVELS[level];
}

#include <avr/pgmspace.h>
const char temp_conf_topic[] PROGMEM  = "homeassistant/sensor/watertank_t/config";
const char temp_conf_payload[] PROGMEM  = R"({"unique_id":"wttemp","name":"Temperature","state_topic":"home/watertank","unit_of_measurement":"°C","icon":"hass:thermometer","value_template":"{{ value_json.temp }}","device":{"identifiers":["231ed3"],"name":"Watertank Sensor","manufacturer":"Tsukihime","model":"nRF24 Watertank Sensor","sw_version":"0.1"}})";

const char lvl_conf_topic[] PROGMEM  = "homeassistant/sensor/watertank_l/config";
const char lvl_conf_payload[] PROGMEM  = R"({"unique_id":"wtlvl","name":"Water level","state_topic":"home/watertank","unit_of_measurement":"%","icon":"mdi:water-percent","value_template":"{{ value_json.lvl }}","device":{"identifiers":["231ed3"]}})";

const char batt_conf_topic[] PROGMEM  = "homeassistant/sensor/watertank_b/config";
const char batt_conf_payload[] PROGMEM  = R"({"unique_id":"wtbatt","name":"Battery","state_topic":"home/watertank","unit_of_measurement":"V","icon":"hass:battery","value_template":"{{ value_json.batt }}","device":{"identifiers":["231ed3"]}})";
    
void identify() {
    RF24MQTT_sendMessage_P(temp_conf_topic, temp_conf_payload, true);
    RF24MQTT_sendMessage_P(lvl_conf_topic, lvl_conf_payload, true);
    RF24MQTT_sendMessage_P(batt_conf_topic, batt_conf_payload, true);
}

void initAll() {
    clock_prescale_set(clock_div_8); // switch clock to 1 MHz
    PRR |= (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2) | // disable all timers
           (1 << PRUSART0) | (1 << PRTWI);                 // disable USART & TWI 

    ACSR |= (1 << ACD);  // disable Analog Comparator
    sleep_bod_disable(); // disable the BOD while sleeping

    // Enable pull-ups on unused I/O pins
    DDRB = 0x00;
    PORTB = 0xFF;
    DDRC = 0x00;
    PORTC = 0xFF;
    DDRD = 0x00;
    PORTD = 0xFF;

    NRF_connect_init();

    RF24_begin();
    RF24_setPALevel(RF24_PA_MAX);
    RF24_enableDynamicPayloads();
    RF24_setDataRate(RF24_1MBPS);
    RF24_setCRCLength(RF24_CRC_8);
    RF24_setChannel(gateway_channel);
    RF24_setAutoAck(true);
    RF24_openWritingPipe(gateway_address);
    RF24_stopListening();

    initWatchdog();
}

int main(void) {
    initAll();
    RF24MQTT_sendShortMessage("watertank", "Start!");
    identify();

    const uint8_t period = 75; // 8sec * 75 = update every 10 min or on level change every 8 sec
    const uint16_t identify_period = 5400; // 8sec * 5400 = update every 12 hours
    uint8_t counter = 0;
    uint16_t identify_counter = 0;
    uint8_t level = 0;
    uint8_t old_level = 0;

    while (1) {
        level = getWatertankLevel();

        if(level != old_level || counter == 0) {
            char buff[40] = "";
            char string[13];

            uint8_t len = int32ToStrFixedPoint(getBatteryVoltage(), string, 3);
            string[len - 1] = 0;
            strcat(buff,"{\"batt\":");
            strcat(buff, string);

            int32ToStrFixedPoint(level, string);
            strcat(buff,",\"lvl\":");
            strcat(buff, string);

            int32ToStrFixedPoint(getInternalTemperature(), string);
            strcat(buff,",\"temp\":");
            strcat(buff, string);
            strcat(buff,"}");

            RF24MQTT_sendMessage("home/watertank", buff, false);
        }

        old_level = level;

        if(++counter >= period) counter = 0;
        if(++identify_counter >= identify_period) {
            identify_counter = 0;
            identify();
        }
        
        enterSleep();
    }
}
