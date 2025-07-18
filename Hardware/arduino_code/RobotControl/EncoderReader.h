#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

class EncoderReader {
public:
    EncoderReader();
    void setup();
    void update(); // Call this in loop()
    long getTicks(uint8_t motorIndex);
    void reset();

private:
    static const uint8_t encoderPins[4];
    long tickCounts[4];
    bool lastStates[4];
    unsigned long lastMicros;
};

#endif