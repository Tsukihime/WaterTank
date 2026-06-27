#ifndef CONFIG_H_
#define CONFIG_H_

#include <avr/pgmspace.h>

#define NODEID      55          // This module's ID (transmitter)
#define NETWORKID   42          // Must match the receiver/gateway
#define GATEWAYID   1           // ID of the gateway/receiver we send to
#define FREQUENCY   RF69_433MHZ
const char ENCRYPTKEY[17] = { "MQTTGateway_42MY" }; // Encryption key (NULL = no encryption)

const uint16_t REFERENCE_VOLTAGE = 1100;

constexpr uint16_t minutes(uint32_t m) {
    return m * 60;
}

constexpr uint16_t hours(uint32_t h) {
    return h * 60 * 60;
}

#define ID_STUB "\xFA\xFB\xFC\xFD\xFE\xFF"
#define BASE_TOPIC "home/watertank_" ID_STUB

char state_topic[] = BASE_TOPIC;
const char id_topic[] PROGMEM = "homeassistant/device/" ID_STUB "/config";
const char id_payload[] PROGMEM = "{"
	R"("dev":{"ids":")" ID_STUB R"(","name":"RFM69 Watertank Sensor","mf":"Tsukihime","mdl":"RFM69 Sensor"},)"
	R"("o":{"name":"Watertank Sensor"},)"
	R"("cmps":{)"
    R"("t":{"uniq_id":")" ID_STUB R"(t","p":"sensor","expire_after":900,"name":"Temperature","dev_cla":"temperature","unit_of_meas":"°C","val_tpl":"{{value_json.temp}}"},)"
    R"("w":{"uniq_id":")" ID_STUB R"(w","p":"sensor","expire_after":900,"name":"TemperatureW","dev_cla":"temperature","unit_of_meas":"°C","val_tpl":"{{value_json.tempw}}"},)"
    R"("p":{"uniq_id":")" ID_STUB R"(p","p":"sensor","expire_after":900,"name":"Pressure","dev_cla":"pressure","unit_of_meas":"hPa","val_tpl":"{{value_json.press}}"},)"
    R"("l":{"uniq_id":")" ID_STUB R"(l","p":"sensor","expire_after":900,"name":"Water level","unit_of_meas":"mm","icon":"mdi:waves-arrow-up","val_tpl":"{{value_json.lvlmm}}"},)"
    R"("b":{"uniq_id":")" ID_STUB R"(b","p":"sensor","expire_after":900,"name":"Battery","dev_cla":"battery","unit_of_meas":"%","val_tpl":"{{value_json.batt}}"},)"
    R"("v":{"uniq_id":")" ID_STUB R"(v","p":"sensor","expire_after":900,"name":"Battery voltage","dev_cla":"voltage","unit_of_meas":"mV","val_tpl":"{{value_json.vbatt}}"},)"
    R"("u":{"uniq_id":")" ID_STUB R"(u","p":"sensor","expire_after":900,"name":"Uplink RSSI","dev_cla":"signal_strength","unit_of_meas":"dBm","val_tpl":"{{value_json.u}}"},)"
    R"("d":{"uniq_id":")" ID_STUB R"(d","p":"sensor","expire_after":900,"name":"Downlink RSSI","dev_cla":"signal_strength","unit_of_meas":"dBm","val_tpl":"{{value_json.d}}"})"
	R"(},)"
	R"("stat_t":")" BASE_TOPIC R"(")"
"}";

#endif /* CONFIG_H_ */
