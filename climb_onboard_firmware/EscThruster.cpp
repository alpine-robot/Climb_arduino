#include "EscThruster.h"

EscThruster::EscThruster(uint8_t pin, int min_us, int max_us, int stop_us)
: pin_(pin),
  minUs_(min_us),
  maxUs_(max_us),
  stopUs_(stop_us),
  currentUs_(stop_us),
  throttle_(0.0f) {}

void EscThruster::begin() {
  esc_.setPeriodHertz(50);
  esc_.attach(pin_, minUs_, maxUs_);
  stop();
}

void EscThruster::writeUs(int us) {
  currentUs_ = us;
  esc_.writeMicroseconds(currentUs_);
}

void EscThruster::setThrottle(float x) {
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  throttle_ = x;

  // 0.0 = 1000 us stop
  // 1.0 = 2000 us full throttle
  const int us = minUs_ + static_cast<int>((maxUs_ - minUs_) * throttle_);

  writeUs(us);
}

void EscThruster::stop() {
  throttle_ = 0.0f;
  writeUs(stopUs_);
}

void EscThruster::refresh() {
  writeUs(currentUs_);
}

void EscThruster::arm(uint32_t arm_ms, uint32_t period_ms) {
  const uint32_t t0 = millis();

  while ((millis() - t0) < arm_ms) {
    stop();
    delay(period_ms);
  }
}