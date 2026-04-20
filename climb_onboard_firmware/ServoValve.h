#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

class ServoValve {
public:
  ServoValve(int pin, int min_us = 500, int max_us = 2500, int frame_us = 20000);

  void begin();
  void setAngle(float deg);

  // compatibility with old code; now no-op
  void sendFrame();

  float angle() const { return angle_deg_; }

private:
  int   pin_;
  int   min_us_;
  int   max_us_;
  int   frame_us_;
  float angle_deg_;
  bool  attached_;
  Servo servo_;

  int angleToUs(float deg) const;
};