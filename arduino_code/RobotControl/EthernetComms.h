#ifndef ETHERNET_COMMS_H
#define ETHERNET_COMMS_H

#include <Arduino.h>
#include <Ethernet.h>

class EthernetComms {
public:
    EthernetComms(IPAddress ip, byte mac[], uint16_t port = 23);
    void begin();
    void update();
    bool hasNewMessage() const;
    String getMessage();

private:
    EthernetServer server;
    EthernetClient client;
    String buffer;
    bool messageAvailable = false;
};

#endif