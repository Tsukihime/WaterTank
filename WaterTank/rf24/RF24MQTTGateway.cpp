#include "RF24MQTTGateway.h"
#include "RF24.h"

#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>

#define RF24_PACKET_SIZE 32

enum class Marker: uint8_t {
	FIRST = 255,
	NEXT = 254,
	LAST = 253
};

struct FirstHeader {
	uint16_t topic_length;
	uint16_t payload_length;
	uint8_t retained;
};

struct FirstPacket : public FirstHeader {
	uint8_t data[RF24_PACKET_SIZE - sizeof(Marker) - sizeof(FirstHeader)];
};

struct NextPacket {
	uint8_t data[RF24_PACKET_SIZE - sizeof(Marker)];
};

struct Packet {
	Marker marker;
	union {
		FirstPacket first;
		NextPacket next;
	};
};

static_assert(sizeof(Marker) == 1, "Marker must be 1 byte");
static_assert(sizeof(FirstHeader) == 5, "FirstHeader must be exactly 5 bytes");
static_assert(sizeof(FirstPacket) == sizeof(FirstHeader) + sizeof(Packet::first.data), "FirstPacket layout mismatch");
static_assert(sizeof(NextPacket) == RF24_PACKET_SIZE - sizeof(Marker), "NextPacket size mismatch");
static_assert(sizeof(Packet) == RF24_PACKET_SIZE, "Packet size mismatch");

bool RF24MQTTGateway::sendToRadio(const char* topic, uint16_t topic_len,
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
            packet.marker = Marker::FIRST;
            packet.first.topic_length = topic_len;
            packet.first.payload_length = payload_len;
            packet.first.retained = retained;			
            data = packet.first.data;
            bytes_to_copy = sizeof(packet.first.data);
        } else {
            packet.marker = Marker::NEXT;			
            data = packet.next.data;
            bytes_to_copy = sizeof(packet.next.data);
        }

        if ((total_size - sent) <= bytes_to_copy) {
            bytes_to_copy = total_size - sent;
            packet.marker = Marker::LAST;
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

        uint16_t data_sz = sizeof(Marker) + bytes_to_copy + (sent == 0 ? sizeof(FirstHeader) : 0);
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
