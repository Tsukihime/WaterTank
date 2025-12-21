#include "RF24MQTTGateway.h"
#include "RF24.h"

#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#include "RF24MQTTProto.h"

bool RF24MQTTGateway::sendToRadio(const char* topic, uint8_t topic_len,
                                  const char* payload, uint16_t payload_len, bool retained,
                                  bool use_callback, gate_callback getdata) {
    Packet packet;
    uint16_t total_size = topic_len + payload_len;
    uint16_t sent = 0;
    bool success = false;
    
    radio.powerUp();
    
    while (sent < total_size) {
        uint16_t bytes_to_copy;
        uint8_t* data;

        if (sent == 0) {
            packet.header.marker = Marker::START;
            packet.header.retained = retained;
            packet.first.topic_length = topic_len;
            packet.first.payload_length = payload_len;
            data = packet.first.data;
            bytes_to_copy = sizeof(FirstPacket::data);
        } else {
            packet.header.marker = Marker::NEXT;
            data = packet.next.data;
            bytes_to_copy = sizeof(NextPacket::data);
        }

        if ((total_size - sent) <= bytes_to_copy) {
            bytes_to_copy = total_size - sent;
            packet.header.marker = (sent == 0) ? Marker::START_STOP : Marker::STOP;
        }

        for (uint8_t i = 0; i < bytes_to_copy; i++) {
            uint16_t offset = sent + i;
            if (offset < topic_len) {
                data[i] = use_callback ? getdata(topic, offset) : topic[offset];
            } else {
                offset -= topic_len;
                data[i] = use_callback ? getdata(payload, offset) : payload[offset];
            }
        }

        uint16_t data_sz = sizeof(PacketHeader) + bytes_to_copy + (sent == 0 ? sizeof(FirstPacket) - sizeof(FirstPacket::data) : 0);
        sent += bytes_to_copy;
        success = radio.write(&packet, data_sz, false);
        if (!success) {
            break;
        }
    }

    radio.powerDown();
    return success;
}

bool RF24MQTTGateway::publish(const char* topic, const char* payload, bool retained) {
    return sendToRadio(topic, strlen(topic), payload, strlen(payload), retained);
}

bool RF24MQTTGateway::publish_P(const char* topic, const char* payload, bool retained) {
    return sendToRadio(topic, strlen_P(topic), payload, strlen_P(payload), retained, true, 
    [](const char* ptr, uintptr_t index) {
        return pgm_read_byte(ptr + index);
    });
}
