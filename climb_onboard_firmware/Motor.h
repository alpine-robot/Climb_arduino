#pragma once
#include <Arduino.h>

class Motor {
public:
  // Keep same constructor style
  Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint32_t pwmHz = 1000);

  void begin();
  void set(float s);
  void stop() { set(0.0f); }
  void setFrequency(uint32_t hz);

  // compatibility with old code; now no-op
  void update();

  float lastCommand() const { return cmd_; }

private:
  uint8_t  rpwm_, lpwm_;
  uint32_t pwmHz_;
  uint8_t  resolutionBits_;
  uint32_t maxDuty_;
  float    cmd_;
  bool     attached_;

  void applyDuty(uint32_t dutyR, uint32_t dutyL);
};