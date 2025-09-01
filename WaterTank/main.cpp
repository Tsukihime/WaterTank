#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#include "Scheduler.h"
#include "utils.h"
#include "bmp280/bmp280.h"

#include "rf24/RF24.h"
#include "rf24/RF24MQTTGateway.h"

BMP280 airSensor(0x77);
BMP280 waterSensor(0x76);

RF24 radio(
    [](uint8_t data) -> uint8_t {
        SPDR = data;
        while (!(SPSR & (1 << SPIF)));
        return SPDR;
    },
    [](bool CSN, bool CE) {
        PORTB = (PORTB & ~((1 << PINB2) | (1 << PINB1))) | (CSN << PINB2) | (CE << PINB1);
    }
);

RF24MQTTGateway mqtt(radio);

uint8_t gateway_address[6] = { "NrfMQ" };
const uint8_t gateway_channel = 0x6f;

void setupWatchdog() {
    wdt_enable(WDTO_8S);
    WDTCSR |= (1 << WDIE); // Enable both interrupt and system reset (combined mode)
}

// WDT Interrupt Is Used To Wake Up CPU From Sleep Mode
EMPTY_INTERRUPT(WDT_vect);

void enterSleep() {
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

void initAll() {
    clock_prescale_set(clock_div_8); // switch clock to 1 MHz
    PRR |= (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2) | // disable all timers
           (1 << PRUSART0);                                // disable USART 

    ACSR |= (1 << ACD);  // disable Analog Comparator
    sleep_bod_disable(); // disable the BOD while sleeping

    // Enable pull-ups on unused I/O pins
    DDRC = 0x00;
    PORTC = 0xFF;
    DDRD = 0x00;
    PORTD = 0xFF;
    
    // RF_init
    PORTB = 0xFF & ~(1 << PINB4);
    DDRB = (1 << PINB1) | (1 << PINB2) | (1 << PINB3) | (1 << PINB5);
    
    PRR &= ~(1 << PRSPI); // Enable SPI
    SPSR = (1 << SPI2X);  // Double SPI speed

    SPCR = 1 << SPE                     /* SPI module enable: enabled */
    | 0 << DORD                  /* Data order: disabled */
    | 1 << MSTR                  /* Master/Slave select: enabled */
    | 0 << CPOL                  /* Clock polarity: disabled */
    | 0 << CPHA                  /* Clock phase: disabled */
    | 0 << SPIE                  /* SPI interrupt enable: disabled */
    | (0 << SPR1) | (0 << SPR0); /* SPI Clock rate selection: fosc/4 */

    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.enableDynamicPayloads();
    radio.setDataRate(RF24_1MBPS);
    radio.setCRCLength(RF24_CRC_8);
    radio.setChannel(gateway_channel);
    radio.setAutoAck(true);
    radio.openWritingPipe(gateway_address);
    radio.stopListening();

    airSensor.init();
    waterSensor.init();

    setupWatchdog();
    sei();
}

void identify() {
    mqtt.publish_P(PSTR("homeassistant/sensor/watertank_t/config"), PSTR(R"({"uniq_id":"wttemp","name":"Temperature","dev_cla":"temperature","stat_t":"home/watertank","unit_of_meas":"°C","val_tpl":"{{value_json.temp}}","dev":{"ids":["231ed3"],"name":"Watertank Sensor","mf":"Tsukihime","mdl":"nRF24 Watertank Sensor"}})"), true);
    mqtt.publish_P(PSTR("homeassistant/sensor/watertank_tw/config"), PSTR(R"({"uniq_id":"wttempw","name":"TemperatureW","dev_cla":"temperature","stat_t":"home/watertank","unit_of_meas":"°C","val_tpl":"{{value_json.tempw}}","dev":{"ids":["231ed3"]}})"), true);
    mqtt.publish_P(PSTR("homeassistant/sensor/watertank_p/config"), PSTR(R"({"uniq_id":"wtpress","name":"Pressure","dev_cla":"pressure","stat_t":"home/watertank","unit_of_meas":"hPa","val_tpl":"{{value_json.press}}","dev":{"ids":["231ed3"]}})"), true);
    mqtt.publish_P(PSTR("homeassistant/sensor/watertank_lmm/config"), PSTR(R"({"uniq_id":"wtlvlmm","name":"Water level","stat_t":"home/watertank","unit_of_meas":"mm","icon":"mdi:waves-arrow-up","val_tpl":"{{value_json.lvlmm}}","dev":{"ids":["231ed3"]}})"), true);
    mqtt.publish_P(PSTR("homeassistant/sensor/watertank_b/config"), PSTR(R"({"uniq_id":"wtbatt","name":"Battery","dev_cla":"battery","stat_t":"home/watertank","unit_of_meas":"%","val_tpl":"{{value_json.batt}}","dev":{"ids":["231ed3"]}})"), true);
    mqtt.publish_P(PSTR("homeassistant/sensor/watertank_v/config"), PSTR(R"({"uniq_id":"wtbattv","name":"Battery voltage","dev_cla":"voltage","stat_t":"home/watertank","unit_of_meas":"mV","val_tpl":"{{value_json.vbatt}}","dev":{"ids":["231ed3"]}})"), true);
}

void measure() {
    const uint8_t MAX_UPDATE_PERIOD = 5;
    static uint8_t update_counter = 0;
    static int16_t old_level = 0;

    waterSensor.takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);
    airSensor.takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);
    int16_t water_level_mm = (((int32_t)waterSensor.getPressurePa()    - (int32_t)airSensor.getPressurePa()) * 100) / 981;

    if(water_level_mm != old_level || update_counter == 0) {
        int16_t voltage = getBatteryVoltage();
        uint8_t battery_level = clamp(round_div(voltage - 2000, 10), 0, 100);
        int16_t water_temp = round_div(waterSensor.getTemperature(), 10);
        int16_t air_temp = round_div(airSensor.getTemperature(), 10);
        
        char json[80] = "";
        char string[13];
        
        int32ToStrFixedPoint(battery_level, string);
        strcat(json, "{\"batt\":");
        strcat(json, string);
        
        int32ToStrFixedPoint(voltage, string);
        strcat(json, ",\"vbatt\":");
        strcat(json, string);
        
        int32ToStrFixedPoint(water_temp, string, 1);
        strcat(json, ",\"tempw\":");
        strcat(json, string);
        
        int32ToStrFixedPoint(air_temp, string, 1);
        strcat(json, ",\"temp\":");
        strcat(json, string);
        
        int32ToStrFixedPoint(airSensor.getPressurePa(), string, 2);
        strcat(json, ",\"press\":");
        strcat(json, string);
        
        int32ToStrFixedPoint(water_level_mm, string);
        strcat(json, ",\"lvlmm\":");
        strcat(json, string);

        strcat(json, "}");

        mqtt.publish("home/watertank", json, false);
        
        old_level = water_level_mm;
        update_counter = 0;
    }
    
    if(++update_counter >= MAX_UPDATE_PERIOD) update_counter = 0;
}

int main(void) {
    initAll();
    identify();
    measure();

    Scheduler::setTimer(identify, 2700, true); // 8sec * 2700 = update every 6 hours
    Scheduler::setTimer(measure, 15, true);    // 8sec * 15   = update every 2 min
    
    while (1) {
        Scheduler::TimerISR();
        while(Scheduler::processTask());        
        enterSleep();
        setupWatchdog(); // After waking up from sleep, restore combined mode
    }
}
