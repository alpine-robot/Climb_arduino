#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class EscThruster {
public:
  explicit EscThruster(uint8_t pin,
                       int min_us = 1000,
                       int max_us = 2000,
                       int stop_us = 1000);

  void begin();

  // comando normalizzato [0..1]
  void setThrottle(float x);

  // stop
  void stop();

  float lastThrottle() const { return throttle_; }

private:
  uint8_t pin_;
  int minUs_;
  int maxUs_;
  int stopUs_;
  float throttle_;
  Servo esc_;
};