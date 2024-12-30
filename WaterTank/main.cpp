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

extern "C" {
	#include "rf24/RF24_spi.h"
	#include "rf24/RF24.h"
}

#include "rf24/MQTTGateway.h"

BMP280 airSensor(0x77);
BMP280 waterSensor(0x76);

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

void identify() {
    MQTTGateway::sendMessage_P(PSTR("homeassistant/sensor/watertank_t/config"), PSTR(R"({"uniq_id":"wttemp","name":"Temperature","dev_cla":"temperature","stat_t":"home/watertank","unit_of_meas":"°C","val_tpl":"{{value_json.temp}}","dev":{"ids":["231ed3"],"name":"Watertank Sensor","mf":"Tsukihime","mdl":"nRF24 Watertank Sensor"}})"), true);
    MQTTGateway::sendMessage_P(PSTR("homeassistant/sensor/watertank_tw/config"), PSTR(R"({"uniq_id":"wttempw","name":"TemperatureW","dev_cla":"temperature","stat_t":"home/watertank","unit_of_meas":"°C","val_tpl":"{{value_json.tempw}}","dev":{"ids":["231ed3"]}})"), true);
    MQTTGateway::sendMessage_P(PSTR("homeassistant/sensor/watertank_p/config"), PSTR(R"({"uniq_id":"wtpress","name":"Pressure","dev_cla":"pressure","stat_t":"home/watertank","unit_of_meas":"hPa","val_tpl":"{{value_json.press}}","dev":{"ids":["231ed3"]}})"), true);
    MQTTGateway::sendMessage_P(PSTR("homeassistant/sensor/watertank_lmm/config"), PSTR(R"({"uniq_id":"wtlvlmm","name":"Water level","stat_t":"home/watertank","unit_of_meas":"mm","icon":"mdi:waves-arrow-up","val_tpl":"{{value_json.lvlmm}}","dev":{"ids":["231ed3"]}})"), true);
    MQTTGateway::sendMessage_P(PSTR("homeassistant/sensor/watertank_b/config"), PSTR(R"({"uniq_id":"wtbatt","name":"Battery","dev_cla":"battery","stat_t":"home/watertank","unit_of_meas":"%","val_tpl":"{{value_json.batt}}","dev":{"ids":["231ed3"]}})"), true);
}

void initAll() {
    clock_prescale_set(clock_div_8); // switch clock to 1 MHz
    PRR |= (1 << PRTIM0) | (1 << PRTIM1) | (1 << PRTIM2) | // disable all timers
           (1 << PRUSART0);                                // disable USART 

    ACSR |= (1 << ACD);  // disable Analog Comparator
    sleep_bod_disable(); // disable the BOD while sleeping

    // Enable pull-ups on unused I/O pins
    DDRB = 0x00;
    PORTB = 0xFF;
    DDRC = 0x00;
    PORTC = 0xFF;
    DDRD = 0x00;
    PORTD = 0xFF;

    RF24_spi_init();

    RF24_begin();
    RF24_setPALevel(RF24_PA_MAX);
    RF24_enableDynamicPayloads();
    RF24_setDataRate(RF24_1MBPS);
    RF24_setCRCLength(RF24_CRC_8);
    RF24_setChannel(gateway_channel);
    RF24_setAutoAck(true);
    RF24_openWritingPipe(gateway_address);
    RF24_stopListening();

	airSensor.init();
	waterSensor.init();

    initWatchdog();
}

void measure() {
	const uint8_t MAX_UPDATE_PERIOD = 5;
	static uint8_t update_counter = 0;
	static int16_t old_level = 0;

	waterSensor.takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);
	airSensor.takeForcedMeasurement(MODE_FORCED, SAMPLING_X2, SAMPLING_X16);
	int16_t water_level_mm = (((int32_t)waterSensor.getPressurePa()
								- (int32_t)airSensor.getPressurePa()) * 100) / 981;	

	if(water_level_mm != old_level || update_counter == 0) {
		uint8_t battery_level = clamp(round_div((int16_t)getBatteryVoltage() - 2000, 10), 0, 100);
		int16_t water_temp = round_div(waterSensor.getTemperature(), 10);
		int16_t air_temp = round_div(airSensor.getTemperature(), 10);
		
		char json[80] = "";
		char string[13];
			
		int32ToStrFixedPoint(battery_level, string);
		strcat(json, "{\"batt\":");
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

		MQTTGateway::sendMessage("home/watertank", json, false);
		
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
    }
}
