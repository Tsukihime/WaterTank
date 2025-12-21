#ifndef RF24MQTTPROTO_H_
#define RF24MQTTPROTO_H_

#define RF24_PACKET_SIZE 32

enum Marker {
    START = 0,
    NEXT = 1,
    STOP = 2,
    START_STOP = 3
};

struct PacketHeader {
    uint8_t marker: 2;
    uint8_t retained: 1;
    uint8_t reserved: 5;
} __attribute__((packed));

struct FirstPacket {
    uint8_t topic_length;
    uint16_t payload_length;
    uint8_t data[RF24_PACKET_SIZE - sizeof(PacketHeader) - 3];
} __attribute__((packed));

struct NextPacket {
    uint8_t data[RF24_PACKET_SIZE - sizeof(PacketHeader)];
} __attribute__((packed));

struct Packet {
    PacketHeader header;
    union {
        FirstPacket first;
        NextPacket next;
    };
} __attribute__((packed));

static_assert(sizeof(PacketHeader) == 1, "Marker must be 1 byte");
static_assert(sizeof(FirstPacket) ==  RF24_PACKET_SIZE - sizeof(PacketHeader), "FirstPacket size mismatch");
static_assert(sizeof(NextPacket) == RF24_PACKET_SIZE - sizeof(PacketHeader), "NextPacket size mismatch");
static_assert(sizeof(Packet) == RF24_PACKET_SIZE, "Packet size mismatch");
static_assert(sizeof(FirstPacket) == sizeof(NextPacket), "Union variants must be same size for layout stability");

#endif /* RF24MQTTPROTO_H_ */
