#include "EncoderReader.h"

const uint8_t EncoderReader::encoderPins[4] = {22, 23, 24, 25};

EncoderReader::EncoderReader() : lastMicros(0) {
    for (int i = 0; i < 4; i++) {
        tickCounts[i] = 0;
        lastStates[i] = HIGH;
    }
}

void EncoderReader::setup() {
    for (int i = 0; i < 4; i++) {
        pinMode(encoderPins[i], INPUT_PULLUP);
    }
}

void EncoderReader::update() {
    unsigned long now = micros();
    if (now - lastMicros < 100) return; // Poll at ~10 kHz
    lastMicros = now;

    for (int i = 0; i < 4; i++) {
        bool state = digitalRead(encoderPins[i]);
        if (lastStates[i] == LOW && state == HIGH) {
            tickCounts[i]++;
        }
        lastStates[i] = state;
    }
}

long EncoderReader::getTicks(uint8_t i) {
    return (i < 4) ? tickCounts[i] : 0;
}

void EncoderReader::reset() {
    for (int i = 0; i < 4; i++) tickCounts[i] = 0;
}