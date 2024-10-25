#include "RF24MQTTGateway.h"
#include "RF24.h"

#include <avr/pgmspace.h>

const uint8_t PACKET_START = 255;
const uint8_t PACKET_NEXT = 254;
const uint8_t PACKET_STOP = 253;

union BigPacket {
    struct {
        uint16_t topic_length;
        uint16_t payload_length;
        uint8_t retained;
        uint8_t data[RF24MQTT_PACKET_SIZE - 5];
    };
    uint8_t raw[RF24MQTT_PACKET_SIZE];
};

void RF24MQTT_sendShortMessage(const char* topic, const char* payload) {
    char buffer[32];
    uint8_t topicLength = strlen(topic);
    uint8_t payloadLength = strlen(payload);
    uint8_t messageLength = topicLength + payloadLength + 2;
    if(messageLength > 32) return;

    buffer[0] = topicLength;
    buffer[1] = payloadLength;
    memcpy(&buffer[2], topic, topicLength);
    memcpy(&buffer[2 + topicLength], payload, payloadLength);

    RF24_powerUp();
    RF24_write(buffer, messageLength, false);
    RF24_powerDown();
}

void RF24MQTT_sendData(uint8_t* data, uint16_t size) {
    RF24_powerUp();
    for(uint16_t i = 0; i < size; i += 31) {
        uint8_t buff[32];
        if(i == 0){
            buff[0] = PACKET_START;
            } else {
            buff[0] = PACKET_NEXT;
        }

        uint8_t len = 31;
        if((size - i) <= len) {
            len = (size - i);
            buff[0] = PACKET_STOP;
        }
        memcpy(&buff[1], &data[i], len);
        RF24_write(buff, len + 1, false);
    }
    RF24_powerDown();
}

void RF24MQTT_sendMessage_P(const char* topic, const char* payload, bool retained) {
    union BigPacket pk;
    pk.retained = retained;
    pk.topic_length = strlen_P(topic);
    pk.payload_length = strlen_P(payload);
    memcpy_P(&pk.data, topic, pk.topic_length);
    memcpy_P(&pk.data[pk.topic_length], payload, pk.payload_length);
    RF24MQTT_sendData(&pk.raw[0], pk.topic_length + pk.payload_length + 5);
}

void RF24MQTT_sendMessage(const char* topic, const char* payload, bool retained) {
    union BigPacket pk;
    pk.retained = retained;
    pk.topic_length = strlen(topic);
    pk.payload_length = strlen(payload);
    memcpy(&pk.data, topic, pk.topic_length);
    memcpy(&pk.data[pk.topic_length], payload, pk.payload_length);
    RF24MQTT_sendData(&pk.raw[0], pk.topic_length + pk.payload_length + 5);
}
