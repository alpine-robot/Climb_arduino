#include "EscThruster.h"

EscThruster::EscThruster(uint8_t pin, int min_us, int max_us, int stop_us)
: pin_(pin),
  minUs_(min_us),
  maxUs_(max_us),
  stopUs_(stop_us),
  throttle_(0.0f) {}

void EscThruster::begin() {
  esc_.setPeriodHertz(50);          // classico segnale ESC
  esc_.attach(pin_, minUs_, maxUs_);
  stop();
}

void EscThruster::setThrottle(float x) {
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;
  throttle_ = x;

  int us = minUs_ + (int)((maxUs_ - minUs_) * throttle_);
  esc_.writeMicroseconds(us);
}

void EscThruster::stop() {
  throttle_ = 0.0f;
  esc_.writeMicroseconds(stopUs_);
}