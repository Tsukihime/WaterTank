#include <RFM69.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#include "config.h"
#include <MQTTGateway.h>
#include <bmp280.h>
#include "utils.h"
#include "Scheduler.h"

#define CS_PIN      PIN_PA6
#define IRQ_PIN     PIN_PA4

RFM69 radio(CS_PIN, IRQ_PIN);

int8_t uplink_rssi = -128;
int8_t downlink_rssi = -128;

MQTTGatewayTransmitter<RF69_MAX_DATA_LEN> mqtt(
    [](const void * data, size_t size) -> bool {
        bool ackReceived = radio.sendWithRetry(GATEWAYID, (const uint8_t*)data, size);
        if (ackReceived && radio.DATALEN >= 1) {
            uplink_rssi = static_cast<int8_t>(radio.DATA[0]);
            downlink_rssi = radio.RSSI;
        }
        return ackReceived;
    }
);

BMP280 airSensor(0x77);
BMP280 waterSensor(0x76);

struct SETTINGS {
    uint16_t magic;
    uint16_t bandgap;
    char id[6];
} settings;

uint16_t getBatteryVoltage() {
    ADC0.CTRLA |= ADC_ENABLE_bm;
    
    ADC0.CTRLC = ADC_PRESC_DIV8_gc      // prescaler /8 ← optimal for 1 MHz (125 kHz)
                | ADC_REFSEL_VDDREF_gc  // Vref = Vcc
                | 0 << ADC_SAMPCAP_bp;  // sample cap off (default)

    ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc; // input = internal bandgap

    _delay_us(200);

    ADC0.COMMAND = ADC_STCONV_bm;
    while (ADC0.COMMAND & ADC_STCONV_bm);

    uint16_t adc_value = ADC0.RES;
    ADC0.CTRLA &= ~ADC_ENABLE_bm;

    if (adc_value == 0) return 0;

    uint32_t voltage = (uint32_t)settings.bandgap * 1024UL / adc_value;
    return (uint16_t)voltage;
}

uint8_t calcBatteryLevel(uint16_t voltage_mv) {
    if (voltage_mv >= 3200) return 100;                            // ≥3.20V = 100%
    if (voltage_mv >= 3000) return 100 - (3200 - voltage_mv) / 20; // 3.20V→100%, 3.00V→90%
    if (voltage_mv >= 2800) return 90  - (3000 - voltage_mv) / 10; // 3.00V→90%,  2.80V→70%
    if (voltage_mv >= 2600) return 70  - (2800 - voltage_mv) / 10; // 2.80V→70%,  2.60V→50%
    if (voltage_mv >= 2400) return 50  - (2600 - voltage_mv) / 10; // 2.60V→50%,  2.40V→30%
    if (voltage_mv >= 2200) return 30  - (2400 - voltage_mv) / 13; // 2.40V→30%,  2.20V→15%
    if (voltage_mv >= 2000) return 15  - (2200 - voltage_mv) / 13; // 2.20V→15%,  2.00V→0%
    return 0;                                                      // <2.00V = 0%
}

uint8_t renderTemplate(const char* _template, uint16_t index) {
    uint8_t c = pgm_read_byte(&_template[index]);
    if (c >= 0xFA) {
        return settings.id[c - 0xFA];
    }    
    return c;
}

void identify() {
    mqtt.send(id_topic, sizeof(id_topic) - 1,
              id_payload, sizeof(id_payload) - 1, true, renderTemplate);
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
        uint8_t battery_level = calcBatteryLevel(voltage);
        int16_t water_temp = round_div(waterSensor.getTemperature(), 10);
        int16_t air_temp = round_div(airSensor.getTemperature(), 10);
        
        char json[100] = "";
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

        int32ToStrFixedPoint(downlink_rssi, string);
        strcat(json, ",\"d\":");
        strcat(json, string);

        int32ToStrFixedPoint(uplink_rssi, string);
        strcat(json, ",\"u\":");
        strcat(json, string);

        strcat(json, "}");

        mqtt.publish(state_topic, json, false);
        
        old_level = water_level_mm;
        update_counter = 0;
    }
    
    if(++update_counter >= MAX_UPDATE_PERIOD) update_counter = 0;
}

void generateUID(char uid[6]) {
    const uint8_t* sernum = (const uint8_t*)&SIGROW.SERNUM0;
    uint32_t hash = 5381;  // initial djb2 value
    for (uint8_t i = 0; i < 10; i++) {
        hash = ((hash << 5) + hash) ^ sernum[i];
    }

    for (int8_t i = 5; i >= 0; i--) {
        uint8_t nibble = hash & 0xF;
        uid[i] = (nibble < 10) ? (nibble + '0') : (nibble - 10 + 'A');
        hash >>= 4;
    }
}

ISR(RTC_PIT_vect) {
    RTC.PITINTFLAGS = RTC_PI_bm; // clear interrupt flag
}

void setupRTC_PIT() {
    while (RTC.STATUS > 0);
    RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;
    RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc // 32768 / 32768 = 1 second
                   | RTC_PITEN_bm;        // enable PIT
    RTC.PITINTCTRL = RTC_PI_bm;           // enable interrupt
}

void sleep_delay(uint16_t seconds) {
    uint8_t miso_backup = PORTA.PIN2CTRL;
    uint8_t irq_backup = PORTA.PIN4CTRL;
    PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;

    while (seconds-- > 0) {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_mode();
    }

    PORTA.PIN2CTRL = miso_backup;
    PORTA.PIN4CTRL = irq_backup;
}

void setupRadio() {
    while(!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
        sleep_delay(5);
    }
    radio.setPowerLevel(31);
    radio.encrypt(ENCRYPTKEY);
    //radio.setFrequency(433920000); //set frequency to some custom frequency
    radio.sleep();
}

void setupHardware() {
    // === 1. Unused pins: disable digital input buffer ===
    PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

    // === 2. Timers ===
    TCA0.SINGLE.CTRLA = 0;
    TCB0.CTRLA     = 0;
    TCB1.CTRLA     = 0;
    //TCD0.CTRLA     = 0; // required for core operation

    // === 3. Communication peripherals ===
    USART0.CTRLB = 0;     // disable completely
    //SPI0.CTRLA = 0;       // required for RFM69 radio
    //TWI0.CTRLA = 0;       // required for BMP280 sensor

    // === 4. Analog blocks ===
    ADC0.CTRLA = 0;       // disable before sleep (even though we measure battery)
    ADC1.CTRLA = 0;

    AC0.CTRLA = 0;
    AC1.CTRLA = 0;
    AC2.CTRLA = 0;

    DAC0.CTRLA = 0;
    DAC1.CTRLA = 0;
    DAC2.CTRLA = 0;
    
    VREF.CTRLA = (VREF.CTRLA & ~VREF_ADC0REFSEL_gm) | VREF_ADC0REFSEL_1V1_gc;

    // === 5. Configurable Custom Logic ===
    CCL.CTRLA = 0;
}

void loadSettings() {
   const uint16_t MAGIC = 0xC0DE;

    eeprom_read_block(&settings, 0, sizeof(settings));
    if (settings.magic != MAGIC) { // load defaults
        settings.magic = MAGIC;
        settings.bandgap = REFERENCE_VOLTAGE;
        generateUID(settings.id);
        eeprom_write_block(&settings, 0, sizeof(settings));
    }

    // Update state topic id
    memcpy(state_topic + sizeof(state_topic) - sizeof(settings.id) - 1, settings.id, sizeof(settings.id));
}

void setup() {
    bool isExternalReset = RSTCTRL.RSTFR & (RSTCTRL_EXTRF_bm |
                                            RSTCTRL_UPDIRF_bm |
                                            RSTCTRL_PORF_bm |
                                            RSTCTRL_SWRF_bm);
    RSTCTRL.RSTFR = RSTCTRL.RSTFR;
    sei();

    setupHardware();
    setupRTC_PIT();
    loadSettings();
    setupRadio();

    airSensor.begin();
    waterSensor.begin();
    airSensor.setSampling(MODE_FORCED, SAMPLING_X2, SAMPLING_X16, FILTER_OFF, STANDBY_MS_1);
    waterSensor.setSampling(MODE_FORCED, SAMPLING_X2, SAMPLING_X16, FILTER_OFF, STANDBY_MS_1);

    identify();
    measure();

    Scheduler::setTimer(identify, hours(6), true);
    Scheduler::setTimer(measure, minutes(1), true);
}

void loop() {
    Scheduler::TimerISR();
    while(Scheduler::processTask());        
    sleep_delay(1);
}
