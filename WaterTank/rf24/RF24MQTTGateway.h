#ifndef RF24MQTTGATEWAY_H_
#define RF24MQTTGATEWAY_H_

#include <stdbool.h>
#include "RF24.h"


typedef uint8_t (*gate_callback)(const char*, uintptr_t index);

class RF24MQTTGateway {
private:
    RF24& radio;
    
public:
    RF24MQTTGateway(RF24& radio) : radio(radio) {}
    bool publish_P(const char* topic, const char* payload, bool retained);
    bool publish(const char* topic, const char* payload, bool retained);
    
    bool sendToRadio(const char* topic, uint16_t topic_len,
                     const char* payload, uint16_t payload_len, bool retained,
                     bool use_callback = false, gate_callback getdata = nullptr);
};

#endif /* RF24MQTTGATEWAY_H_ */
