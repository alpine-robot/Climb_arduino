#include "Motor.h"

#if defined(ARDUINO_ARCH_ESP32)
  #include <esp32-hal-ledc.h>
#endif

Motor::Motor(uint8_t rpwmPin, uint8_t lpwmPin, uint32_t pwmHz)
: rpwm_(rpwmPin),
  lpwm_(lpwmPin),
  pwmHz_(pwmHz ? pwmHz : 1000),
  resolutionBits_(10),
  maxDuty_(0),
  cmd_(0.0f),
  attached_(false)
{}

void Motor::begin() {
  pinMode(rpwm_, OUTPUT);
  pinMode(lpwm_, OUTPUT);
  digitalWrite(rpwm_, LOW);
  digitalWrite(lpwm_, LOW);

#if defined(ARDUINO_ARCH_ESP32)
  bool okR = ledcAttach(rpwm_, pwmHz_, resolutionBits_);
  bool okL = ledcAttach(lpwm_, pwmHz_, resolutionBits_);
  attached_ = okR && okL;
  maxDuty_ = (1UL << resolutionBits_) - 1UL;
#else
  attached_ = false;
#endif

  stop();
}

void Motor::set(float s) {
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  cmd_ = s;

  if (!attached_) {
    digitalWrite(rpwm_, LOW);
    digitalWrite(lpwm_, LOW);
    return;
  }

  const float mag = fabsf(cmd_);
  const uint32_t duty = (uint32_t)(mag * (float)maxDuty_);

  if (cmd_ > 0.0f) {
    applyDuty(duty, 0);
  } else if (cmd_ < 0.0f) {
    applyDuty(0, duty);
  } else {
    applyDuty(0, 0);
  }
}

void Motor::setFrequency(uint32_t hz) {
  if (hz < 100) hz = 100;
  if (hz > 5000) hz = 5000;
  pwmHz_ = hz;

#if defined(ARDUINO_ARCH_ESP32)
  if (!attached_) return;

  uint32_t f1 = ledcChangeFrequency(rpwm_, pwmHz_, resolutionBits_);
  uint32_t f2 = ledcChangeFrequency(lpwm_, pwmHz_, resolutionBits_);

  if (f1 == 0 || f2 == 0) {
    attached_ = false;
    digitalWrite(rpwm_, LOW);
    digitalWrite(lpwm_, LOW);
    return;
  }

  set(cmd_);
#endif
}

void Motor::update() {
  // no-op: PWM is already maintained in hardware
}

void Motor::applyDuty(uint32_t dutyR, uint32_t dutyL) {
#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(rpwm_, dutyR);
  ledcWrite(lpwm_, dutyL);
#else
  digitalWrite(rpwm_, dutyR ? HIGH : LOW);
  digitalWrite(lpwm_, dutyL ? HIGH : LOW);
#endif
}