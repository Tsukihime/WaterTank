#ifndef RF24MQTTGATEWAY_H_
#define RF24MQTTGATEWAY_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef RF24MQTT_PACKET_SIZE
    #define RF24MQTT_PACKET_SIZE 400
#endif

void RF24MQTT_sendShortMessage(const char* topic, const char* payload);

void RF24MQTT_sendData(uint8_t* data, uint16_t size);

void RF24MQTT_sendMessage_P(const char* topic, const char* payload, bool retained);

void RF24MQTT_sendMessage(const char* topic, const char* payload, bool retained);

#endif /* RF24MQTTGATEWAY_H_ */
