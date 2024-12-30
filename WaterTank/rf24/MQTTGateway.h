#ifndef MQTTGATEWAY_H_
#define MQTTGATEWAY_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifndef RF24MQTT_PACKET_SIZE
    #define RF24MQTT_PACKET_SIZE 400
#endif

class MQTTGateway {
    public:
        static void sendShortMessage(const char* topic, const char* payload);
        static void sendData(uint8_t* data, uint16_t size);
        static void sendMessage_P(const char* topic, const char* payload, bool retained);
        static void sendMessage(const char* topic, const char* payload, bool retained);

    private:
        static const uint8_t PACKET_START = 255;
        static const uint8_t PACKET_NEXT = 254;
        static const uint8_t PACKET_STOP = 253;

        union BigPacket {
            struct {
                uint16_t topic_length;
                uint16_t payload_length;
                uint8_t retained;
                uint8_t data[RF24MQTT_PACKET_SIZE - 5];
            };
            uint8_t raw[RF24MQTT_PACKET_SIZE];
        };
};

#endif /* MQTTGATEWAY_H_ */