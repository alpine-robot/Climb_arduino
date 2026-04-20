#include "Brake.h"

Brake::Brake(uint8_t relayPin) : relayPin(relayPin) {}

void Brake::begin() {
    pinMode(relayPin, OUTPUT);
    digitalWrite(relayPin, LOW); // start with brake engaged
}

void Brake::setBrakeEngaged(bool engaged) {
    digitalWrite(relayPin, engaged ? LOW : HIGH);
}

bool Brake::isBrakeEngaged() const {
    return digitalRead(relayPin) == LOW;
}