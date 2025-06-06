#include "EncoderReader.h"

const uint8_t EncoderReader::encoderPins[4] = {22, 23, 24, 25};

EncoderReader::EncoderReader() {
    for (int i = 0; i < 4; i++) {
        tickCounts[i] = 0;
        lastStates[i] = LOW;
    }
}

void EncoderReader::setup() {
    for (uint8_t pin : encoderPins) {
        pinMode(pin, INPUT);
    }
}

void EncoderReader::update() {
    for (int i = 0; i < 4; i++) {
        bool state = digitalRead(encoderPins[i]);
        if (state != lastStates[i]) {
            lastStates[i] = state;
            if (state == HIGH) tickCounts[i]++;
        }
    }
}

long EncoderReader::getTicks(uint8_t i) {
    return (i < 4) ? tickCounts[i] : 0;
}

void EncoderReader::reset() {
    for (int i = 0; i < 4; i++) tickCounts[i] = 0;
}