#ifndef RF24MQTTGATEWAY_H_
#define RF24MQTTGATEWAY_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "RF24.h"

class RF24MQTTGateway {
public:
    RF24MQTTGateway(RF24& radio) : radio(radio) {}
    void publish_P(const char* topic, const char* payload, bool retained);
    void publish(const char* topic, const char* payload, bool retained);

private:
    RF24& radio;
    void sendToRadio(const char* topic, const char* payload, bool retained, bool progmem);
};


#endif /* RF24MQTTGATEWAY_H_ */
