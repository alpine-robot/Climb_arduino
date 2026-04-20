#pragma once
#include <Arduino.h>

class Brake {
public:
    Brake(uint8_t relayPin);
    void begin();

    void setBrakeEngaged(bool engaged);  // true = engage brake
    bool isBrakeEngaged() const;         // reads pin state

private:
    uint8_t relayPin;
};