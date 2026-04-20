#include "ServoValve.h"

ServoValve::ServoValve(int pin, int min_us, int max_us, int frame_us)
  : pin_(pin),
    min_us_(min_us),
    max_us_(max_us),
    frame_us_(frame_us),
    angle_deg_(0.0f),
    attached_(false)
{}

void ServoValve::begin() {
  servo_.setPeriodHertz(1000000 / frame_us_);   // usually 50 Hz for 20000 us
  attached_ = servo_.attach(pin_, min_us_, max_us_);
  if (attached_) {
    setAngle(angle_deg_);
  }
}

void ServoValve::setAngle(float deg) {
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 90.0f) deg = 90.0f;
  angle_deg_ = deg;

  if (!attached_) return;

  const int us = angleToUs(angle_deg_);
  servo_.writeMicroseconds(us);
}

int ServoValve::angleToUs(float deg) const {
  return (int)(min_us_ + (deg / 90.0f) * (max_us_ - min_us_));
}

// Old API compatibility: PWM is already maintained in hardware.
void ServoValve::sendFrame() {
  // no-op
}