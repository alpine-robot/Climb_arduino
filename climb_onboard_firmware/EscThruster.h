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

  void setThrottle(float x);

  void stop();

  void refresh();

  void arm(uint32_t arm_ms = 3000, uint32_t period_ms = 20);

  float lastThrottle() const { return throttle_; }
  int lastPulseUs() const { return currentUs_; }

private:
  void writeUs(int us);

  uint8_t pin_;
  int minUs_;
  int maxUs_;
  int stopUs_;
  int currentUs_;
  float throttle_;
  Servo esc_;
};