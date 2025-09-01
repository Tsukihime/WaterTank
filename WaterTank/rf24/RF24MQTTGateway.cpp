#include "RF24MQTTGateway.h"
#include "RF24.h"

#include <avr/pgmspace.h>

#define RF24_PACKET_SIZE 32

const uint8_t PACKET_START = 255;
const uint8_t PACKET_NEXT = 254;
const uint8_t PACKET_STOP = 253;

union Packet {
    struct {
        struct {
            uint8_t marker;                 // Packet marker (START/NEXT/STOP)
            uint16_t topic_length;          // Length of the topic
            uint16_t payload_length;        // Length of the payload
            uint8_t retained;               // Retained flag
        } header;                           // Packet header
        uint8_t data[RF24_PACKET_SIZE - 6]; // Data field (26 bytes)
    } first;
    struct {
        struct {
            uint8_t marker;                  // Packet marker (START/NEXT/STOP)
        } header;                            // Packet header
        uint8_t data[RF24_PACKET_SIZE - 1];  // Data field (31 bytes)
    } next;
    uint8_t raw[RF24_PACKET_SIZE];           // Raw packet buffer (32 bytes)
};

void RF24MQTTGateway::sendToRadio(const char* topic, const char* payload, bool retained, bool progmem) {
    if (!topic) return;

    union Packet packet;
    uint16_t topic_len = progmem ? strlen_P(topic) : strlen(topic);
    uint16_t payload_len = progmem ? strlen_P(payload) : strlen(payload);
    uint16_t total_size = topic_len + payload_len;

    radio.powerUp();

    for (uint16_t i = 0; i < total_size;) {
        uint16_t bytes_to_copy;
        uint8_t* data_ptr;

        if (i == 0) {
            packet.first.header.marker = PACKET_START;
            packet.first.header.topic_length = topic_len;
            packet.first.header.payload_length = payload_len;
            packet.first.header.retained = retained;
            data_ptr = packet.first.data;
            bytes_to_copy = sizeof(packet.first.data);
        } else {
            packet.next.header.marker = PACKET_NEXT;
            data_ptr = packet.next.data;
            bytes_to_copy = sizeof(packet.next.data);
        }

        if ((total_size - i) <= bytes_to_copy) {
            bytes_to_copy = total_size - i;
            packet.next.header.marker = PACKET_STOP;
        }

        for (uint8_t k = 0; k < bytes_to_copy; k++) {
            if (i + k < topic_len) {
                data_ptr[k] = progmem ?
                    pgm_read_byte(&topic[i + k]) :
                    topic[i + k];
            } else {
                data_ptr[k] = progmem ?
                    pgm_read_byte(&payload[i + k - topic_len]) :
                    payload[i + k - topic_len];
            }
        }

        uint16_t data_sz = bytes_to_copy + (i == 0 ? sizeof(packet.first.header) : sizeof(packet.next.header));
        i += bytes_to_copy;
        bool is_ok = radio.write(packet.raw, data_sz, false);
        if(!is_ok) {
            break;
        }
    }

    radio.powerDown();
}

void RF24MQTTGateway::publish(const char* topic, const char* payload, bool retained) {
    sendToRadio(topic, payload, retained, false);
}

void RF24MQTTGateway::publish_P(const char* topic, const char* payload, bool retained) {
    sendToRadio(topic, payload, retained, true);
}
