#include "EthernetComms.h"

EthernetComms::EthernetComms(IPAddress ip, byte mac[], uint16_t port)
: server(port), buffer("") {
    Ethernet.begin(mac, ip);
}

void EthernetComms::begin() {
    server.begin();
    Serial.println("Ethernet server started");
    Serial.print("IP Address: ");
    Serial.println(Ethernet.localIP());
}

void EthernetComms::update() {
    if (!client || !client.connected()) {
        client = server.available();
        if (client) {
            Serial.println("Client connected.");
            buffer = "";
        }
        return;
    }

    while (client.available()) {
        char c = client.read();
        if (c == '\n') {
            buffer.trim();
            messageAvailable = true;
        } else {
            buffer += c;
        }
    }
}

bool EthernetComms::hasNewMessage() const {
    return messageAvailable;
}

String EthernetComms::getMessage() {
    messageAvailable = false;
    String msg = buffer;
    buffer = "";
    return msg;
}