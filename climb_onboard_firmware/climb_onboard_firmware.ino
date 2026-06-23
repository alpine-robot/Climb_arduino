/*
README — ESP32-S3 Servo + BTS7960 Motor Tester (Software-PWM Motor)
===================================================================

What this does
--------------
- Drives TWO servo valves on GPIO 35 and 36 using your ServoValve class (bit-banged 50 Hz).
- Drives ONE DC motor via a BTS7960 (RPWM=GPIO 37, LPWM=GPIO 38) using your *software-PWM* Motor class.
- Serial + ESP-NOW command console to set angles and motor duty.
- 100 Hz ESP-NOW telemetry sender: epoch_ms,<imu1_csv_wo_nl>,<imu2_csv_wo_nl>

Requirements
------------
- Arduino IDE with "esp32 by Espressif Systems" installed.
  * File → Preferences → Additional Boards Manager URLs:
      https://dl.espressif.com/dl/package_esp32_index.json
  * Tools → Board → ESP32 Arduino → **ESP32S3 Dev Module**
- Files in your project:
  * ServoValve.h / ServoValve.cpp  (0–90° mapping, .begin(), .setAngle(), .sendFrame())
  * Motor.h / Motor.cpp            (begin(), setFrequency(hz), set(val), stop(), update())
  * Movella.h / Movella.cpp        (imu.begin(...), .update(), .printCSV(Print&))
  * EspNow.h / EspNow.cpp          (from our previous step)

Wiring (default pins)
---------------------
- ServoValve1 signal → GPIO 35
- ServoValve2 signal → GPIO 36
- BTS7960 RPWM → GPIO 37
- BTS7960 LPWM → GPIO 38
- Common GND between ESP32-S3, servos, and BTS7960 power.

Serial Commands
---------------
- s1 <deg>     → set valve1 angle in degrees (0..90)
- s2 <deg>     → set valve2 angle in degrees (0..90)
- m<val>       → motor command in [-1..1], e.g. m-1, m0, m0.25, m1
- mf <hz>      → set motor PWM to arbitrary frequency (e.g., "mf 200")
- mstop        → stop motor (0 duty)
- status       → print current angles, motor command, espnow tx count
- help         → reprint help
*/

#include <Arduino.h>
#include <WiFi.h>
#include "ServoValve.h"
#include "Motor.h"
#include "Movella.h"
#include "EspNow.h"
#include <esp_mac.h>  // at top, with other includes

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif

#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

// ── Peer (of dongle) MAC address — CHANGE THIS ───────────────────────────────────
uint8_t DONGLE_MAC[6] = { 0x50, 0x78, 0x7D, 0x16, 0xA9, 0x0C };

// Set to -1 if positive IMU pitch means nose-up.
// Set to +1 if positive IMU pitch means nose-down.
static constexpr float PITCH_PID_OUTPUT_SIGN = -1.0f;

static constexpr float YAW_PID_OUTPUT_SIGN = 1.0f;

static bool pitchHoldActiveBand = false;
static bool yawHoldActiveBand = false;
static bool  yawHoldInit       = false;
static float yawLastRad        = 0.0f;
static float yawRateFilt       = 0.0f;

// Soglie yaw robuste
volatile float yawDeadOutDeg      = 8.0f;  // parte solo sopra questa soglia
volatile float yawDriftRateDegS   = 1.5f;   // sotto questa velocità considero drift lento
volatile float yawDriftBandDeg    = 30.0f;  // compenso drift solo entro questa banda
volatile float yawDriftAlpha      = 0.0015f;


// Runtime serial mode.
// Default = Arduino Serial Monitor / manual bench mode.
// ROS will switch this at runtime by sending: "ros on".
volatile bool rosSerialBridgeMode = false;

// Default off, otherwise Arduino Serial Monitor gets flooded.
// ROS enables it at runtime.
volatile bool usbTelemetryEnabled = false;

// 100 Hz / 5 = 20 Hz when enabled.
static constexpr uint8_t USB_TELEMETRY_DIV_DEFAULT = 5;
volatile uint8_t usbTelemetryDiv = USB_TELEMETRY_DIV_DEFAULT;
static uint8_t usbTelemCounter = 0;

// Keep false for the USB-serial test.
// You can still turn this into a runtime command later if needed.
volatile bool espnowTelemetryEnabled = false;

// ── Pins (adjust if needed) ───────────────────────────────────────────────────
static constexpr uint8_t SERVO1_PIN = 35;
static constexpr uint8_t SERVO2_PIN = 36;
static constexpr uint8_t RPWM_PIN   = 37;   // BTS7960 RPWM
static constexpr uint8_t LPWM_PIN   = 38;   // BTS7960 LPWM

// Thruster pins: questi sono quelli verificati nel tester LEDC su ESP32-S3.
static constexpr uint8_t PIN_T1 = 12;
static constexpr uint8_t PIN_T2 = 13;
static constexpr uint8_t PIN_T3 = 14;
static constexpr uint8_t PIN_T4 = 15;
static constexpr uint8_t PIN_T5 = 39;   // top pitch thruster
static constexpr uint8_t PIN_T6 = 21;   // bottom pitch thruster

// LEDC channels: devono essere espliciti e unici, altrimenti i PWM si copiano tra pin.
static constexpr uint8_t CH_T1 = 0;
static constexpr uint8_t CH_T2 = 1;
static constexpr uint8_t CH_T3 = 2;
static constexpr uint8_t CH_T4 = 3;
static constexpr uint8_t CH_T5 = 4;
static constexpr uint8_t CH_T6 = 5;

// ESC pulse values
static constexpr int ESC_MIN_US  = 1000;
static constexpr int ESC_MAX_US  = 2000;
static constexpr int ESC_STOP_US = 1000;

// PWM settings for ESCs
static constexpr uint32_t ESC_PWM_FREQ_HZ   = 50;
static constexpr uint8_t  ESC_PWM_RES_BITS  = 14;
static constexpr uint32_t ESC_PWM_PERIOD_US = 1000000UL / ESC_PWM_FREQ_HZ;

static inline uint32_t escUsToDuty(int us) {
  if (us < 0) us = 0;
  if (us > (int)ESC_PWM_PERIOD_US) us = ESC_PWM_PERIOD_US;

  const uint32_t maxDuty = (1UL << ESC_PWM_RES_BITS) - 1;
  return (uint32_t)(((uint64_t)us * maxDuty + ESC_PWM_PERIOD_US / 2) / ESC_PWM_PERIOD_US);
}

static inline bool escAttachPwm(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcAttachChannel(pin, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS, channel);
#else
  ledcSetup(channel, ESC_PWM_FREQ_HZ, ESC_PWM_RES_BITS);
  ledcAttachPin(pin, channel);
  return true;
#endif
}

static inline bool escWritePwm(uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  return ledcWriteChannel(channel, duty);
#else
  ledcWrite(channel, duty);
  return true;
#endif
}

// Sostituisce il vecchio EscThruster basato su ESP32Servo.
// Nome diverso per evitare conflitti se EscThruster.h/.cpp restano nella cartella del progetto.
// Mantiene la stessa API usata dal resto del programma: begin(), setThrottle(), stop(), lastThrottle().
class LedcThruster {
public:
  LedcThruster(uint8_t pin,
              uint8_t channel,
              int min_us = ESC_MIN_US,
              int max_us = ESC_MAX_US,
              int stop_us = ESC_STOP_US)
  : pin_(pin),
    channel_(channel),
    minUs_(min_us),
    maxUs_(max_us),
    stopUs_(stop_us),
    currentUs_(stop_us),
    throttle_(0.0f),
    attached_(false) {}

  bool begin() {
    pinMode(pin_, OUTPUT);
    attached_ = escAttachPwm(pin_, channel_);
    stop();
    return attached_;
  }

  void setThrottle(float x) {
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    throttle_ = x;
    const int us = minUs_ + static_cast<int>((maxUs_ - minUs_) * throttle_);
    writeUs(us);
  }

  void stop() {
    throttle_ = 0.0f;
    writeUs(stopUs_);
  }

  void refresh() {
    writeUs(currentUs_);
  }

  void arm(uint32_t arm_ms = 3000, uint32_t period_ms = 20) {
    const uint32_t t0 = millis();
    while ((millis() - t0) < arm_ms) {
      stop();
      delay(period_ms);
    }
  }

  float lastThrottle() const { return throttle_; }
  int lastPulseUs() const { return currentUs_; }
  uint8_t pin() const { return pin_; }
  uint8_t channel() const { return channel_; }
  bool attached() const { return attached_; }

private:
  void writeUs(int us) {
    currentUs_ = us;
    const uint32_t duty = escUsToDuty(currentUs_);
    const bool ok = escWritePwm(channel_, duty);
    if (!ok) {
      Serial.print("ERROR ledcWrite on thruster pin=");
      Serial.print(pin_);
      Serial.print(" ch=");
      Serial.println(channel_);
    }
  }

  uint8_t pin_;
  uint8_t channel_;
  int minUs_;
  int maxUs_;
  int stopUs_;
  int currentUs_;
  float throttle_;
  bool attached_;
};

// ── Objects ───────────────────────────────────────────────────────────────────
ServoValve ServoValve1(SERVO1_PIN);
ServoValve ServoValve2(SERVO2_PIN);

LedcThruster thr1(PIN_T1, CH_T1);
LedcThruster thr2(PIN_T2, CH_T2);
LedcThruster thr3(PIN_T3, CH_T3);
LedcThruster thr4(PIN_T4, CH_T4);
LedcThruster thr5(PIN_T5, CH_T5);   // upper pitch
LedcThruster thr6(PIN_T6, CH_T6);   // lower pitch

// Software-PWM Motor (keep low-moderate due to blocking servo frames)
Motor motor(RPWM_PIN, LPWM_PIN, 1000);

// Movella IMUs on two UARTs
HardwareSerial Xsens1(2);  // UART2: RX=16, TX=17
HardwareSerial Xsens2(1);  // UART1: RX=5,  TX=4
Movella imu1(Xsens1, 1);
Movella imu2(Xsens2, 2);

// ── Thruster timeout ─────────────────
volatile uint32_t lastThrCmdMs = 0;
static constexpr uint32_t THR_TIMEOUT_MS = 500;         // external cyclic commands, e.g. WRC / ROS / ESP-NOW
static constexpr uint32_t MANUAL_THR_TIMEOUT_MS = 5000; // bench commands from Serial: t1..t6, THR, pth
static constexpr uint32_t SERIAL_IDLE_COMMAND_MS = 80;  // accepts Serial Monitor set to "No line ending"

// ── ESP-NOW telemetry health / auto-recovery ─────────────────
volatile bool g_espnowHealthy = false;
volatile uint32_t g_espnowConsecutiveSendFails = 0;
volatile uint32_t g_espnowLastOkTxMs = 0;
volatile uint32_t g_espnowLastReinitMs = 0;
static constexpr uint32_t ESPNOW_REINIT_COOLDOWN_MS = 2000;
static constexpr uint32_t ESPNOW_STALE_MS = 1000;
static constexpr uint32_t ESPNOW_FAIL_THRESHOLD = 5;


volatile float cmdFx = 0.0f;   // body-frame force x request from high level
volatile float cmdFy = 0.0f;   // body-frame force y request from high level
volatile float cmdMz = 0.0f;   // open-loop yaw moment request from high level

volatile bool propCtrlEnabled = false;  // lateral wrench from ROS/ESP-NOW enabled
volatile bool yawCtlEnabled   = false;  // IMU-based yaw hold enabled
volatile bool manualThrusterMode = false; // true for THR / tN / pth bench commands until timeout

// Final thruster configuration (already agreed with wiring):
// T1 = Y1 = CW   , Pack A
// T2 = Y2 = CCW  , Pack A
// T3 = Y3 = CCW  , Pack B
// T4 = Y4 = CW   , Pack B
// T5 = P1 = CW   , Pack A  (pitch +)
// T6 = P2 = CCW  , Pack B  (pitch -)
// Yaw +  => T1 + T3
// Yaw -  => T2 + T4

static constexpr float FX_GAIN = 1.0f;
static constexpr float FY_GAIN = 1.0f;
static constexpr float MZ_GAIN = 1.0f;
static constexpr float THR_MAX = 1.0f;
static constexpr float PITCH_MIN_ACTIVE = 0.80f;
static constexpr float YAW_MIN_ACTIVE   = 0.60f;

static constexpr float PITCH_RAMP_STEP   = 0.10f;
static constexpr float LATERAL_RAMP_STEP = 0.08f;
// assist mode ----------------------------------------------------
volatile bool assistEnabled = false;
volatile uint32_t assistEndMs = 0;
volatile float assistUmax = 1.0f;
volatile float assistKp = 3.0f;
volatile float assistKd = 0.25f;

// ── Pitch hold (onboard, time-critical) ─────────────────────────
volatile bool  pitchCtlEnabled = false;
volatile float pitchRefRad     = 0.0f;
volatile float pitchKp         = 1.2f;
volatile float pitchKi         = 0.0f;
volatile float pitchKd         = 0.05f;
volatile float pitchUmax       = 0.6f;
volatile float pitchDeadDeg    = 2.0f;   // "dritto" band
volatile float pitchSafeDeg    = 45.0f;
static float pitchPidI         = 0.0f;
static float pitchPrevE        = 0.0f;

// ── Yaw hold (onboard, time-critical) ───────────────────────────
volatile float yawRefRad       = 0.0f;
volatile float yawKp           = 0.9f;
volatile float yawKd           = 0.04f;
volatile float yawUmax         = 0.45f;
volatile float yawDeadDeg      = 3.0f;

// Yaw robusto contro drift lento IMU
volatile float yawDeadOutExtraDeg = 8.0f;   // con ydb=10 parte sopra circa 18 deg


static float yawPrevE             = 0.0f;

static float lastPitchCmd      = 0.0f;
static float lastYawCmd        = 0.0f;

static inline float applyMinActiveSigned(float u, float minActive, float maxActive) {
  maxActive = clampf(maxActive, 0.0f, 1.0f);
  minActive = clampf(minActive, 0.0f, maxActive);

  u = clampf(u, -maxActive, maxActive);

  if (fabsf(u) < 1e-5f) {
    return 0.0f;
  }

  float a = fabsf(u);
  if (a < minActive) {
    a = minActive;
  }

  return (u > 0.0f) ? a : -a;
}
// Attitude for propeller stabilization must come from the BODY IMU.
// In alpine_odometry_node.py the mapping is:
//   IMU1 = rope IMU
//   IMU2 = body IMU
// If your physical wiring is the opposite, swap this helper accordingly.
static inline void getAttitudeQuat(float q[4]) {
  imu2.getQuaternion(q);   // body IMU
}

// ── Robust line reader for Serial ─────────────────────────────────────────────
// Works with Arduino Serial Monitor set to Newline, Both NL & CR, or No line ending.
String line;
static uint32_t lastSerialCharMs = 0;
static constexpr size_t SERIAL_MAX_LINE_LEN = 180;

bool readLine(String& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastSerialCharMs = millis();

    if (c == '\r' || c == '\n') {
      out.trim();

      if (out.length() == 0) {
        out = "";
        return false;
      }

      return true;
    }

    out += c;

    if (out.length() > SERIAL_MAX_LINE_LEN) {
      out = "";

      if (!rosSerialBridgeMode) {
        Serial.println("ERR: serial command too long, buffer cleared.");
      }

      return false;
    }
  }

  // Solo per Arduino Serial Monitor / test manuali.
  // In ROS mode NO: il nodo ROS manda già newline.
  if (!rosSerialBridgeMode &&
      out.length() > 0 &&
      (millis() - lastSerialCharMs) >= SERIAL_IDLE_COMMAND_MS) {
    out.trim();

    if (out.length() == 0) {
      out = "";
      return false;
    }

    return true;
  }

  return false;
}

void printHelp() {
  Serial.println(F(
    "Commands:\n"
    "  s1 <deg>                 - set valve1 angle (0..90)\n"
    "  s2 <deg>                 - set valve2 angle (0..90)\n"
    "  THR,t1,t2,t3,t4,t5,t6    - set all thrusters duty\n"
    "  t1 <val> ... t6 <val>    - set one thruster duty without changing the others\n"
    "  pth <val>                - manual pitch thrusters command in [-1..1]\n"
    "  pitch                    - print current pitch angle\n"
    "  yaw                      - print current yaw angle\n"
    "  apon / apoff             - enable/disable onboard pitch hold\n"
    "  ayon / ayoff             - enable/disable onboard yaw hold\n"
    "  atton / attoff           - enable/disable both pitch+yaw hold\n"
    "  apzero / ayzero          - set current pitch/yaw as reference\n"
    "  attzero                  - set both references to current attitude\n"
    "  pid <Kp> <Ki> <Kd>       - set pitch controller gains\n"
    "  ypid <Kp> <Kd>           - set yaw controller gains\n"
    "  umax <0..1>              - set max pitch thruster command\n"
    "  yumax <0..1>             - set max yaw thruster command\n"
    "  pdb <deg> / ydb <deg>    - set pitch/yaw deadbands in degrees\n"
    "  m<val>                   - motor command in [-1..1]\n"
    "  mf <hz>                  - set motor PWM frequency\n"
    "  mstop                    - stop motor\n"
    "  WRC,fx,fy,mz             - set lateral wrench command (open-loop bias)\n"
    "  pron / proff             - enable/disable lateral high-level wrench\n"
    "  ext                      - test motor extend\n"
    "  ret                      - test motor retract\n"
    "  status                   - print current state\n"
    "  arm / disarm             - send min throttle / stop all ESCs\n"
    "  stop                     - stop all thrusters\n"
    "  help                     - show this help\n"
  ));
}

// ── Helpers & forward declarations ────────────────────────────────────────────
// Sink that satisfies Stream& (required by Movella::printCSV)
struct StringStreamSink : public Stream {
  String s;

  // ---- Stream required methods (not used for input) ----
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}

  // ---- Write (used by printCSV) ----
  size_t write(uint8_t c) override { s += char(c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    s.reserve(s.length() + n);
    for (size_t i = 0; i < n; ++i) s += char(b[i]);
    return n;
  }

  void clear() { s.remove(0); }
};


static inline void rstrip_nl(String& s) {
  while (s.endsWith("\n") || s.endsWith("\r")) s.remove(s.length() - 1);
}

// Shared command parser (Serial + ESP-NOW)
void handleCommandLine(const String& cmd);

// Build one combined CSV line:
// epoch_ms,<imu1_csv_wo_nl>,<imu2_csv_wo_nl>\n
String buildDualImuCsv();

// 100 Hz TX task pinned to APP CPU (keeps timing despite blocking in loop)
void EspNowTxTask(void* arg);
bool ensureEspNowHealthy();





static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float wrapPi(float a) {
  while (a >  3.1415926f) a -= 2.0f * 3.1415926f;
  while (a < -3.1415926f) a += 2.0f * 3.1415926f;
  return a;
}

// Euler extraction from quaternion (assume q0=w, q1=x, q2=y, q3=z)
static float pitchFromQuatWXYZ(float w, float x, float y, float z) {
  float sinp = 2.0f * (w*y - z*x);
  sinp = clampf(sinp, -1.0f, 1.0f);
  return asinf(sinp);
}

static float yawFromQuatWXYZ(float w, float x, float y, float z) {
  float siny = 2.0f * (w*z + x*y);
  float cosy = 1.0f - 2.0f * (y*y + z*z);
  return atan2f(siny, cosy);
}
static float pitchPidStep(float pitchRad, float dt) {
  float e = pitchRefRad - pitchRad;

  const float deadIn  = pitchDeadDeg * (3.1415926f / 180.0f);
  const float deadOut = (pitchDeadDeg + 3.0f) * (3.1415926f / 180.0f);

  float ae = fabsf(e);

  if (ae < deadIn) {
    pitchHoldActiveBand = false;
  } else if (ae > deadOut) {
    pitchHoldActiveBand = true;
  }

  if (!pitchHoldActiveBand) {
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    return 0.0f;
  }

  float P = pitchKp * e;
  pitchPidI += pitchKi * e * dt;
  pitchPidI = clampf(pitchPidI, -0.5f, 0.5f);

  float D = pitchKd * (e - pitchPrevE) / dt;
  pitchPrevE = e;

  float u = P + pitchPidI + D;
float uSat = clampf(u, -pitchUmax, pitchUmax);

if (u != uSat) pitchPidI *= 0.95f;

return applyMinActiveSigned(uSat, PITCH_MIN_ACTIVE, pitchUmax);
}

static float yawPidStep(float yawRad, float dt) {
  if (dt <= 1e-5f) {
    return 0.0f;
  }

  if (!yawHoldInit) {
    yawHoldInit = true;
    yawLastRad = yawRad;
    yawRateFilt = 0.0f;
    yawPrevE = 0.0f;
    yawHoldActiveBand = false;
    return 0.0f;
  }

  // Velocità yaw misurata
  float dy = wrapPi(yawRad - yawLastRad);
  yawLastRad = yawRad;

  float yawRate = dy / dt;
  yawRateFilt = 0.92f * yawRateFilt + 0.08f * yawRate;

  float e = wrapPi(yawRefRad - yawRad);

  const float deadIn    = yawDeadDeg * (3.1415926f / 180.0f);
  const float deadOut   = (yawDeadDeg + yawDeadOutExtraDeg) * (3.1415926f / 180.0f);
  const float driftRate = yawDriftRateDegS * (3.1415926f / 180.0f);
  const float driftBand = yawDriftBandDeg * (3.1415926f / 180.0f);

  // Se lo yaw si sposta lentamente e siamo in una banda ragionevole,
  // lo considero drift IMU, non rotazione reale.
  if (!yawHoldActiveBand &&
      fabsf(e) < driftBand &&
      fabsf(yawRateFilt) < driftRate) {

    yawRefRad = wrapPi(yawRefRad - yawDriftAlpha * e);
    e = wrapPi(yawRefRad - yawRad);
  }

  // Deadband con isteresi
  if (fabsf(e) < deadIn) {
    yawHoldActiveBand = false;
    yawPrevE = 0.0f;
    return 0.0f;
  }

  if (fabsf(e) > deadOut) {
    yawHoldActiveBand = true;
  }

  if (!yawHoldActiveBand) {
    yawPrevE = 0.0f;

    // Smorzamento solo se ruota davvero veloce anche nella zona intermedia
    if (fabsf(yawRateFilt) > 3.0f * driftRate) {
      float uDamp = -yawKd * yawRateFilt;
     return applyMinActiveSigned(uDamp, YAW_MIN_ACTIVE, yawUmax);
    }

    return 0.0f;
  }

  // P sulla posizione + D sulla velocità reale
  float u = yawKp * e - yawKd * yawRateFilt;
  u = clampf(u, -yawUmax, yawUmax);

  return applyMinActiveSigned(u, YAW_MIN_ACTIVE, yawUmax);
}

// opzionale: rampetta per non dare step bruschi agli ESC
static float pitchTopCmd = 0.0f;
static float pitchBotCmd = 0.0f;
static float latCmd1 = 0.0f;
static float latCmd2 = 0.0f;
static float latCmd3 = 0.0f;
static float latCmd4 = 0.0f;

static inline float rampTowards(float current, float target, float step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

static inline bool isManualThrusterActive(uint32_t now = millis()) {
  return manualThrusterMode && ((now - lastThrCmdMs) <= MANUAL_THR_TIMEOUT_MS);
}

static inline void enterManualThrusterMode() {
  manualThrusterMode = true;
  propCtrlEnabled = false;
  yawCtlEnabled = false;
  pitchCtlEnabled = false;
  assistEnabled = false;
  cmdFx = 0.0f;
  cmdFy = 0.0f;
  cmdMz = 0.0f;
  lastThrCmdMs = millis();
}

static inline void setPitchThrustersDirect(float pitchCmd) {
  pitchCmd = clampf(pitchCmd, -1.0f, 1.0f);

  float t5 = 0.0f;
  float t6 = 0.0f;

  if (pitchCmd > 0.0f) {
    t5 = pitchCmd;
  } else if (pitchCmd < 0.0f) {
    t6 = -pitchCmd;
  }

  pitchTopCmd = t5;
  pitchBotCmd = t6;

  thr5.setThrottle(t5);
  thr6.setThrottle(t6);
}

static inline bool setSingleThrusterManual(uint8_t number, float throttle) {
  throttle = clampf(throttle, 0.0f, 1.0f);

  switch (number) {
    case 1: latCmd1 = throttle; thr1.setThrottle(throttle); break;
    case 2: latCmd2 = throttle; thr2.setThrottle(throttle); break;
    case 3: latCmd3 = throttle; thr3.setThrottle(throttle); break;
    case 4: latCmd4 = throttle; thr4.setThrottle(throttle); break;
    case 5: thr5.setThrottle(throttle); pitchTopCmd = throttle; break;
    case 6: thr6.setThrottle(throttle); pitchBotCmd = throttle; break;
    default: return false;
  }

  return true;
}

static inline void printThrusterAttachStatus(const char* name, uint8_t pin, uint8_t channel, bool attached) {
  Serial.print("Attach ");
  Serial.print(name);
  Serial.print(" pin=");
  Serial.print(pin);
  Serial.print(" ch=");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.println(attached ? "OK" : "FAIL");
}



static inline float pitchActiveTarget(float x) {
  x = clampf(x, 0.0f, 1.0f);

  const float maxPitch = clampf(pitchUmax, 0.0f, 1.0f);
  const float minPitch = clampf(PITCH_MIN_ACTIVE, 0.0f, maxPitch);

  if (x <= 1e-5f) {
    return 0.0f;
  }

  if (x < minPitch) {
    x = minPitch;
  }

  return clampf(x, 0.0f, maxPitch);
}

static inline void setPitchThrusters(float pitchCmd) {
  pitchCmd = clampf(pitchCmd, -1.0f, 1.0f);

  float t5_target = 0.0f;
  float t6_target = 0.0f;

  if (pitchCmd > 0.0f) {
    t5_target = pitchActiveTarget(pitchCmd);
  } else if (pitchCmd < 0.0f) {
    t6_target = pitchActiveTarget(-pitchCmd);
  }

  // T5 active
  if (t5_target > 0.0f) {
    pitchBotCmd = 0.0f;

    if (pitchTopCmd < PITCH_MIN_ACTIVE) {
      pitchTopCmd = PITCH_MIN_ACTIVE;   // parte subito da 0.8
    } else {
      pitchTopCmd = rampTowards(pitchTopCmd, t5_target, PITCH_RAMP_STEP);
    }

    pitchTopCmd = clampf(pitchTopCmd, PITCH_MIN_ACTIVE, pitchUmax);
  }

  // T6 active
  else if (t6_target > 0.0f) {
    pitchTopCmd = 0.0f;

    if (pitchBotCmd < PITCH_MIN_ACTIVE) {
      pitchBotCmd = PITCH_MIN_ACTIVE;   // parte subito da 0.8
    } else {
      pitchBotCmd = rampTowards(pitchBotCmd, t6_target, PITCH_RAMP_STEP);
    }

    pitchBotCmd = clampf(pitchBotCmd, PITCH_MIN_ACTIVE, pitchUmax);
  }

  // dentro deadband: spento
  else {
    pitchTopCmd = 0.0f;
    pitchBotCmd = 0.0f;
  }

  thr5.setThrottle(pitchTopCmd);
  thr6.setThrottle(pitchBotCmd);
}

static inline void setLateralThrustersFromWrench(float fx, float fy, float mz) {
  fx = clampf(fx, -1.0f, 1.0f);
  fy = clampf(fy, -1.0f, 1.0f);
  mz = clampf(mz, -1.0f, 1.0f);

  float t1 = 0.0f;
  float t2 = 0.0f;
  float t3 = 0.0f;
  float t4 = 0.0f;

  // Yaw allocation according to agreed propeller pairs:
  // -Mz => right => T1 + T3
  // +Mz => left  => T2 + T4
  if (mz > 0.0f) {
    t2 += MZ_GAIN * mz;
    t4 += MZ_GAIN * mz;
  } else if (mz < 0.0f) {
    t1 += MZ_GAIN * (-mz);
    t3 += MZ_GAIN * (-mz);
  }

  // Tangential motion on the wall (keep as config-driven bias, tune after real mounting)
   // Lateral translation on the wall:
  // -Fy => left  => T1 + T2
  // +Fy => right => T3 + T4
  if (fy > 0.0f) {
    t3 += FY_GAIN * fy;
    t4 += FY_GAIN * fy;
  } else if (fy < 0.0f) {
    t1 += FY_GAIN * (-fy);
    t2 += FY_GAIN * (-fy);
  }

  // Normal push/pull bias (placeholder until full geometry allocation is identified)
  if (fx > 0.0f) {
    t2 += FX_GAIN * fx;
    t3 += FX_GAIN * fx;
  } else if (fx < 0.0f) {
    t1 += FX_GAIN * (-fx);
    t4 += FX_GAIN * (-fx);
  }

  t1 = clampf(t1, 0.0f, THR_MAX);
  t2 = clampf(t2, 0.0f, THR_MAX);
  t3 = clampf(t3, 0.0f, THR_MAX);
  t4 = clampf(t4, 0.0f, THR_MAX);

  latCmd1 = rampTowards(latCmd1, t1, LATERAL_RAMP_STEP);
  latCmd2 = rampTowards(latCmd2, t2, LATERAL_RAMP_STEP);
  latCmd3 = rampTowards(latCmd3, t3, LATERAL_RAMP_STEP);
  latCmd4 = rampTowards(latCmd4, t4, LATERAL_RAMP_STEP);

  thr1.setThrottle(latCmd1);
  thr2.setThrottle(latCmd2);
  thr3.setThrottle(latCmd3);
  thr4.setThrottle(latCmd4);
}

static inline void stopLateralThrusters() {
  thr1.stop();
  thr2.stop();
  thr3.stop();
  thr4.stop();
  latCmd1 = latCmd2 = latCmd3 = latCmd4 = 0.0f;
}

static inline void stopPitchThrusters() {
  thr5.stop();
  thr6.stop();
  pitchTopCmd = 0.0f;
  pitchBotCmd = 0.0f;
}

static inline void stopAllThrusters() {
  stopLateralThrusters();
  stopPitchThrusters();
}

static inline void armAllThrusters(uint32_t armMs = 3000, uint32_t periodMs = 20) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < armMs) {
    stopAllThrusters();
    delay(periodMs);
  }
}


// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);
  
  lastThrCmdMs = millis();

  // --- Valves ---
  ServoValve1.begin();
  ServoValve2.begin();
  
  thr1.begin(); printThrusterAttachStatus("T1", PIN_T1, CH_T1, thr1.attached());
  thr2.begin(); printThrusterAttachStatus("T2", PIN_T2, CH_T2, thr2.attached());
  thr3.begin(); printThrusterAttachStatus("T3", PIN_T3, CH_T3, thr3.attached());
  thr4.begin(); printThrusterAttachStatus("T4", PIN_T4, CH_T4, thr4.attached());
  thr5.begin(); printThrusterAttachStatus("T5", PIN_T5, CH_T5, thr5.attached());
  thr6.begin(); printThrusterAttachStatus("T6", PIN_T6, CH_T6, thr6.attached());

  // Robust ESC arming: send min-throttle for a few seconds at 50 Hz.
  // This avoids the periodic beep / micro-movement state typical of "not armed" ESCs.
  Serial.println("Arming ESCs at minimum throttle...");
  armAllThrusters(3000, 20);
  lastThrCmdMs = millis();
  Serial.println("ESCs armed.");

  ServoValve1.setAngle(0);
  ServoValve2.setAngle(0);
  for (int i = 0; i < 5; ++i) {
    ServoValve1.sendFrame();
    ServoValve2.sendFrame();
    delay(20);
  }

  // --- Motor ---
  motor.begin();
  motor.set(0.0f);

  // --- IMUs ---
  imu1.begin(115200, 16, 17);
  imu2.begin(115200, 5, 4);

  // --- ESP-NOW ---
  WiFi.mode(WIFI_STA);                 // required
  uint8_t localMac[6] = {0};
  esp_read_mac(localMac, ESP_MAC_WIFI_STA);

  // Initialize ESP-NOW and start telemetry task even if first init fails.
  g_espnowHealthy = EspNow_init(DONGLE_MAC);
  g_espnowConsecutiveSendFails = 0;
  g_espnowLastOkTxMs = 0;
  g_espnowLastReinitMs = millis();

  Serial.printf(
    "[ESP-NOW] local=%02X:%02X:%02X:%02X:%02X:%02X  "
    "peer=%02X:%02X:%02X:%02X:%02X:%02X  init_ok=%s\n",
    localMac[0],localMac[1],localMac[2],localMac[3],localMac[4],localMac[5],
    DONGLE_MAC[0],DONGLE_MAC[1],DONGLE_MAC[2],DONGLE_MAC[3],DONGLE_MAC[4],DONGLE_MAC[5],
    g_espnowHealthy ? "true" : "false"
  );

  EspNow_setCommandCallback(handleCommandLine);

  // Always launch telemetry TX task; it will self-heal/reinit if ESP-NOW is down.
  BaseType_t ok = xTaskCreatePinnedToCore(
      EspNowTxTask, "espnow_tx", 4096, nullptr, 1, nullptr, APP_CPU_NUM);
  if (ok != pdPASS) Serial.println("[ESP-NOW] ERROR: TX task create failed!");

 
  Serial.println("Ready.");
printHelp();
Serial.print("> ");
}
// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Parse Serial commands
  //Serial.println("hello!");
 if (readLine(line)) {
  String cmd = line;
  cmd.trim();
  line = "";

  if (cmd.length() > 0) {
    if (!rosSerialBridgeMode) {
      Serial.print("RX: ");
      Serial.println(cmd);
    }

    handleCommandLine(cmd);
  }

  if (!rosSerialBridgeMode) {
    Serial.print("> ");
  }
}

  // Pump ESP-NOW RX queue → dispatches to handleCommandLine()
  EspNow_loop();

  // Safety timeout for externally requested propeller commands.
  // Manual bench commands use a longer timeout so they do not disappear immediately.
  const uint32_t now = millis();
  const bool manualActive = isManualThrusterActive(now);

  if (manualThrusterMode && !manualActive) {
    manualThrusterMode = false;
    stopAllThrusters();
  }

  if (!manualActive && (now - lastThrCmdMs > THR_TIMEOUT_MS)) {
    cmdFx = 0.0f;
    cmdFy = 0.0f;
    cmdMz = 0.0f;

    if (!yawCtlEnabled && !propCtrlEnabled) {
      stopLateralThrusters();
    }
    if (!pitchCtlEnabled) {
      stopPitchThrusters();
    }
  }

  // Run software PWM for motor as often as possible
  motor.update();
  
  // Refresh valves (bit-bang one ~20 ms frame each)
  // NOTE: These calls block; the 100 Hz TX task keeps running anyway.
  ServoValve1.sendFrame();
  ServoValve2.sendFrame();

  // One extra motor.update() after long blocking frames (optional)
  motor.update();

  

  // IMU CSV is handled by the 100 Hz TX task; avoid double-printing here.
  // if (imu1.update()) imu1.printCSV(Serial);
  // if (imu2.update()) imu2.printCSV(Serial);
}

// ── Command parser used by Serial *and* ESP-NOW ───────────────────────────────
void handleCommandLine(const String& in) {
  String cmd = in;
  cmd.trim();
  String low = cmd;
  low.toLowerCase();

  if (low == "ros on" || low == "roson" || low == "bridge on") {
    rosSerialBridgeMode = true;
    usbTelemetryEnabled = true;
    usbTelemetryDiv = USB_TELEMETRY_DIV_DEFAULT;
    usbTelemCounter = 0;

    Serial.printf("OK ros on usbtelem_div=%u\n", usbTelemetryDiv);
    return;

  } else if (low == "ros off" || low == "rosoff" || low == "console") {
    usbTelemetryEnabled = false;
    rosSerialBridgeMode = false;
    usbTelemCounter = 0;

    Serial.println("OK console mode");
    printHelp();
    Serial.print("> ");
    return;

  } else if (low == "usbtelem off" || low == "usbtelem 0" || low == "telem off") {
    usbTelemetryEnabled = false;
    usbTelemCounter = 0;

    Serial.println("OK usbtelem off");
    return;

  } else if (low.startsWith("usbtelem ")) {
    String arg = low.substring(String("usbtelem ").length());
    arg.trim();

    int div = arg.toInt();
    if (div < 1) div = USB_TELEMETRY_DIV_DEFAULT;
    if (div > 100) div = 100;

    usbTelemetryDiv = (uint8_t)div;
    usbTelemetryEnabled = true;
    usbTelemCounter = 0;

    Serial.printf("OK usbtelem div=%u\n", usbTelemetryDiv);
    return;

  } else if (low == "telem on") {
    usbTelemetryEnabled = true;
    usbTelemetryDiv = USB_TELEMETRY_DIV_DEFAULT;
    usbTelemCounter = 0;

    Serial.printf("OK telem on div=%u\n", usbTelemetryDiv);
    return;
  }

  if (low == "ping") {
    Serial.println("pong");

  } else if (low.startsWith("s1")) {
    float v = cmd.substring(2).toFloat();
    ServoValve1.setAngle(v);
    Serial.printf("Valve1 -> %.1f deg\n", v);

  } else if (low.startsWith("s2")) {
    float v = cmd.substring(2).toFloat();
    ServoValve2.setAngle(v);
    Serial.printf("Valve2 -> %.1f deg\n", v);

  } else if (low.startsWith("thr,")) {
    float t1, t2, t3, t4, t5, t6;
    int matched = sscanf(cmd.c_str(), "THR,%f,%f,%f,%f,%f,%f", &t1, &t2, &t3, &t4, &t5, &t6);
    if (matched != 6) {
      matched = sscanf(cmd.c_str(), "thr,%f,%f,%f,%f,%f,%f", &t1, &t2, &t3, &t4, &t5, &t6);
    }

    if (matched == 6) {
      // Full manual thruster mode for bench testing.
      // The 100 Hz task will not overwrite these outputs until THR_TIMEOUT_MS expires.
      enterManualThrusterMode();

      t1 = clampf(t1, 0.0f, 1.0f);
      t2 = clampf(t2, 0.0f, 1.0f);
      t3 = clampf(t3, 0.0f, 1.0f);
      t4 = clampf(t4, 0.0f, 1.0f);
      t5 = clampf(t5, 0.0f, 1.0f);
      t6 = clampf(t6, 0.0f, 1.0f);

      latCmd1 = t1;
      latCmd2 = t2;
      latCmd3 = t3;
      latCmd4 = t4;
      pitchTopCmd = t5;
      pitchBotCmd = t6;

      thr1.setThrottle(t1);
      thr2.setThrottle(t2);
      thr3.setThrottle(t3);
      thr4.setThrottle(t4);
      thr5.setThrottle(t5);
      thr6.setThrottle(t6);

      Serial.printf("THR -> %.2f %.2f %.2f %.2f %.2f %.2f, manual timeout=%lu ms\n",
                    t1, t2, t3, t4, t5, t6, (unsigned long)MANUAL_THR_TIMEOUT_MS);
    } else {
      Serial.println("ERR: usage THR,t1,t2,t3,t4,t5,t6");
    }

  } else if (low.startsWith("t")) {
    char commandLetter = 0;
    int thrusterNumber = 0;
    float throttle = 0.0f;

    int itemsRead = sscanf(cmd.c_str(), "%c%d %f", &commandLetter, &thrusterNumber, &throttle);
    if (itemsRead != 3) {
      itemsRead = sscanf(cmd.c_str(), "%c%d,%f", &commandLetter, &thrusterNumber, &throttle);
    }

    if (itemsRead == 3 && (commandLetter == 't' || commandLetter == 'T')) {
      enterManualThrusterMode();
      if (setSingleThrusterManual((uint8_t)thrusterNumber, throttle)) {
        lastThrCmdMs = millis();
        Serial.printf("T%d -> %.2f, manual timeout=%lu ms\n",
                      thrusterNumber,
                      clampf(throttle, 0.0f, 1.0f),
                      (unsigned long)MANUAL_THR_TIMEOUT_MS);
      } else {
        Serial.println("ERR: use t1..t6, example: t1 0.3");
      }
    } else {
      Serial.println("ERR: usage t1 0.3");
    }

  } else if (low.startsWith("pth ")) {
  float u = 0.0f;
  int n = sscanf(cmd.c_str(), "pth %f", &u);
  if (n == 1) {
    enterManualThrusterMode();
    stopLateralThrusters();
    setPitchThrustersDirect(u);
    lastThrCmdMs = millis();
    Serial.printf("Pitch thrusters -> %.2f, manual timeout=%lu ms\n",
                  clampf(u, -1.0f, 1.0f), (unsigned long)MANUAL_THR_TIMEOUT_MS);
  } else {
    Serial.println("Usage: pth <-1..1>");
  }

  } else if (low.startsWith("wrc,")) {
  float fx, fy, mz;
  int matched = sscanf(cmd.c_str(), "WRC,%f,%f,%f", &fx, &fy, &mz);
  if (matched != 3) {
    matched = sscanf(cmd.c_str(), "wrc,%f,%f,%f", &fx, &fy, &mz);
  }

  if (matched == 3) {
    manualThrusterMode = false;
    propCtrlEnabled = true;
    cmdFx = clampf(fx, -1.0f, 1.0f);
    cmdFy = clampf(fy, -1.0f, 1.0f);
    cmdMz = clampf(mz, -1.0f, 1.0f);
    lastThrCmdMs = millis();

    setLateralThrustersFromWrench(cmdFx, cmdFy, cmdMz);

    Serial.printf("WRC -> Fx=%.2f Fy=%.2f Mz=%.2f\n", cmdFx, cmdFy, cmdMz);
  } else {
    Serial.println("ERR: usage WRC,fx,fy,mz");
  }

} else if (low == "stop") {
    manualThrusterMode = false;
    pitchCtlEnabled = false;
    yawCtlEnabled = false;
    propCtrlEnabled = false;
    assistEnabled = false;
    cmdFx = 0.0f;
    cmdFy = 0.0f;
    cmdMz = 0.0f;
    stopAllThrusters();
    Serial.println("All thrusters stopped.");

} else if (low == "mstop") {
    motor.stop();
    Serial.println("Motor stopped.");

  } else if (low.startsWith("mf ")) {
    String arg = cmd.substring(2);
    arg.trim();
    uint32_t hz = (uint32_t)arg.toInt();
    if (hz < 100) hz = 100;
    if (hz > 2000) hz = 2000;
    motor.setFrequency(hz);
    Serial.printf("Motor PWM set to %u Hz.\n", hz);

  } else if (low == "ext") {
    pitchCtlEnabled = false;
    motor.set(+0.5f);
    Serial.println("EXT test: motor +0.5 (should extend toward the black ball)");

  } else if (low == "ret") {
    pitchCtlEnabled = false;
    motor.set(-0.5f);
    Serial.println("RET test: motor -0.5 (should retract)");

  } else if (low == "pitch") {
    float q[4];
    getAttitudeQuat(q);
    float pitchRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    float pitchDeg = pitchRad * 180.0f / 3.1415926f;
    Serial.printf("Pitch = %.2f deg\n", pitchDeg);

  } else if (low == "yaw") {
    float q[4];
    getAttitudeQuat(q);
    float yawRad = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    float yawDeg = yawRad * 180.0f / 3.1415926f;
    Serial.printf("Yaw = %.2f deg\n", yawDeg);

  } else if (low == "apon") {
    manualThrusterMode = false;
    pitchCtlEnabled = true;
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    Serial.println("AutoPitch ON (pitch thrusters)");

  } else if (low == "apoff") {
  manualThrusterMode = false;
  pitchCtlEnabled = false;
  assistEnabled = false;
  stopPitchThrusters();
  motor.stop();
  Serial.println("AutoPitch OFF");

  } else if (low == "ayon") {
    manualThrusterMode = false;
    yawCtlEnabled = true;
    yawPrevE = 0.0f;
    Serial.println("AutoYaw ON (lateral thrusters)");

  } else if (low == "ayoff") {
    manualThrusterMode = false;
    yawCtlEnabled = false;
    if (!propCtrlEnabled) stopLateralThrusters();
    Serial.println("AutoYaw OFF");

  } else if (low == "atton") {
    manualThrusterMode = false;
    pitchCtlEnabled = true;
    yawCtlEnabled = true;
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    yawPrevE = 0.0f;
    Serial.println("Attitude hold ON (pitch + yaw)");

  } else if (low == "attoff") {
    manualThrusterMode = false;
    pitchCtlEnabled = false;
    yawCtlEnabled = false;
    assistEnabled = false;
    stopAllThrusters();
    Serial.println("Attitude hold OFF");

  } else if (low == "pron") {
  manualThrusterMode = false;
  propCtrlEnabled = true;
  Serial.println("Lateral prop control ON");
} else if (low == "proff") {
  manualThrusterMode = false;
  propCtrlEnabled = false;
  cmdFx = 0.0f;
  cmdFy = 0.0f;
  cmdMz = 0.0f;
  stopLateralThrusters();
  Serial.println("Lateral prop control OFF");
  }else if (low == "apzero") {
    float q[4];
    getAttitudeQuat(q);
    pitchRefRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    Serial.println("AutoPitch reference set (apzero)");

  } else if (low == "ayzero") {
    float q[4];
    getAttitudeQuat(q);
    yawRefRad = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    yawPrevE = 0.0f;
    yawHoldInit = false;
    yawHoldActiveBand = false;
    yawRateFilt = 0.0f;
    Serial.println("AutoYaw reference set (ayzero)");

  } else if (low == "attzero") {
    float q[4];
    getAttitudeQuat(q);
    pitchRefRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    yawRefRad = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    pitchPidI = 0.0f;
    pitchPrevE = 0.0f;
    yawPrevE = 0.0f;
    yawHoldInit = false;
    yawHoldActiveBand = false;
    yawRateFilt = 0.0f;
    Serial.println("Pitch+Yaw references set from current attitude");

  } else if (low.startsWith("pid ")) {
    float p, i, d;
    int n = sscanf(cmd.c_str(), "pid %f %f %f", &p, &i, &d);
    if (n == 3) {
      pitchKp = p;
      pitchKi = i;
      pitchKd = d;
      Serial.printf("PID set: Kp=%.3f Ki=%.3f Kd=%.3f\n", pitchKp, pitchKi, pitchKd);
    } else {
      Serial.println("Usage: pid <Kp> <Ki> <Kd>");
    }

  } else if (low.startsWith("umax ")) {
    float x;
    int n = sscanf(cmd.c_str(), "umax %f", &x);
    if (n == 1) {
      pitchUmax = clampf(x, 0.0f, 1.0f);
      Serial.printf("Pitch uMax=%.2f\n", pitchUmax);
    } else {
      Serial.println("Usage: umax <0..1>");
    }

  } else if (low.startsWith("ypid ")) {
    float p, d;
    int n = sscanf(cmd.c_str(), "ypid %f %f", &p, &d);
    if (n == 2) {
      yawKp = p;
      yawKd = d;
      Serial.printf("Yaw PID set: Kp=%.3f Kd=%.3f\n", yawKp, yawKd);
    } else {
      Serial.println("Usage: ypid <Kp> <Kd>");
    }

  } else if (low.startsWith("yumax ")) {
    float x;
    int n = sscanf(cmd.c_str(), "yumax %f", &x);
    if (n == 1) {
      yawUmax = clampf(x, 0.0f, 1.0f);
      Serial.printf("Yaw uMax=%.2f\n", yawUmax);
    } else {
      Serial.println("Usage: yumax <0..1>");
    }

  } else if (low.startsWith("pdb ")) {
    float x;
    int n = sscanf(cmd.c_str(), "pdb %f", &x);
    if (n == 1) {
      pitchDeadDeg = clampf(x, 0.0f, 30.0f);
      Serial.printf("Pitch deadband=%.2f deg\n", pitchDeadDeg);
    } else {
      Serial.println("Usage: pdb <deg>");
    }

  } else if (low.startsWith("ydb ")) {
    float x;
    int n = sscanf(cmd.c_str(), "ydb %f", &x);
    if (n == 1) {
      yawDeadDeg = clampf(x, 0.0f, 45.0f);
      Serial.printf("Yaw deadband=%.2f deg\n", yawDeadDeg);
    } else {
      Serial.println("Usage: ydb <deg>");
    }

  } else if (low.startsWith("assist ")) {
    manualThrusterMode = false;
    int ms = 0;
    int n = sscanf(cmd.c_str(), "assist %d", &ms);
    if (n == 1) {
      if (ms < 50) ms = 50;
      if (ms > 2000) ms = 2000;
      assistEnabled = true;
      assistEndMs = millis() + (uint32_t)ms;
      pitchPidI = 0.0f;
      pitchPrevE = 0.0f;
      Serial.printf("Assist ON for %d ms\n", ms);
    } else {
      Serial.println("Usage: assist <ms 50..2000>");
    }

  } else if (low.startsWith("m")) {
    if (pitchCtlEnabled) {
      Serial.println("AutoPitch ON: usa 'apoff' per controllo manuale motore.");
      return;
    }

    String arg = cmd.substring(1);
    arg.replace(" ", "");
    if (arg.length() == 0) {
      Serial.println("Usage: m<val> where val in [-1..1]");
    } else {
      float v = arg.toFloat();
      motor.set(v);
      Serial.printf("Motor -> %.3f\n", v);
    }

  } else if (low == "status") {
  Serial.println("--- STATUS ---");
  Serial.printf("Motor duty cmd: %.3f\n", motor.lastCommand());
  Serial.printf("cmdFx=%.2f cmdFy=%.2f cmdMz=%.2f manualThr=%d manualActive=%d\n",
                cmdFx, cmdFy, cmdMz, (int)manualThrusterMode, (int)isManualThrusterActive());
  Serial.printf("THR_TIMEOUT_MS=%lu MANUAL_THR_TIMEOUT_MS=%lu\n",
                (unsigned long)THR_TIMEOUT_MS, (unsigned long)MANUAL_THR_TIMEOUT_MS);
  Serial.printf("pitchCtl=%d yawCtl=%d propCtrl=%d pitchRef=%.2fdeg yawRef=%.2fdeg pDead=%.2f yDead=%.2f\n",
                (int)pitchCtlEnabled, (int)yawCtlEnabled, (int)propCtrlEnabled,
                pitchRefRad * 180.0f / 3.1415926f, yawRefRad * 180.0f / 3.1415926f,
                pitchDeadDeg, yawDeadDeg);
  Serial.printf("lastPitchCmd=%.2f lastYawCmd=%.2f\n", lastPitchCmd, lastYawCmd);
  Serial.printf("T1=%.2f T2=%.2f T3=%.2f T4=%.2f T5=%.2f T6=%.2f\n",
    thr1.lastThrottle(), thr2.lastThrottle(), thr3.lastThrottle(),
    thr4.lastThrottle(), thr5.lastThrottle(), thr6.lastThrottle());
  Serial.printf("T_us: %d %d %d %d %d %d\n",
    thr1.lastPulseUs(), thr2.lastPulseUs(), thr3.lastPulseUs(),
    thr4.lastPulseUs(), thr5.lastPulseUs(), thr6.lastPulseUs());
    
  Serial.printf("ESP-NOW tx_count: %lu\n", (unsigned long)EspNow_txCount());

  } else if (low == "arm") {
    manualThrusterMode = false;
    Serial.println("Arming ESCs...");
    armAllThrusters(3000, 20);
    lastThrCmdMs = millis();
    Serial.println("ESCs armed.");

  } else if (low == "disarm") {
    manualThrusterMode = false;
    pitchCtlEnabled = false;
    yawCtlEnabled = false;
    propCtrlEnabled = false;
    assistEnabled = false;
    cmdFx = 0.0f;
    cmdFy = 0.0f;
    cmdMz = 0.0f;
    stopAllThrusters();
    Serial.println("ESCs forced to minimum throttle.");

  } else if (low == "help" || low == "?") {
    printHelp();

  } else if (low.length() > 0) {
    Serial.println("Unknown. Type 'help'.");
  }
}

// ── Build dual-IMU CSV line ───────────────────────────────────────────────────
bool ensureEspNowHealthy() {
  const uint32_t now = millis();

  const bool stale = (g_espnowLastOkTxMs > 0) && ((now - g_espnowLastOkTxMs) > ESPNOW_STALE_MS);
  const bool tooManyFails = (g_espnowConsecutiveSendFails >= ESPNOW_FAIL_THRESHOLD);

  if (g_espnowHealthy && !stale && !tooManyFails) return true;
  if ((now - g_espnowLastReinitMs) < ESPNOW_REINIT_COOLDOWN_MS) return g_espnowHealthy;

  g_espnowLastReinitMs = now;
  Serial.printf("[ESP-NOW] reinit requested (healthy=%s stale=%s fails=%lu)\n",
                g_espnowHealthy ? "true" : "false",
                stale ? "true" : "false",
                (unsigned long)g_espnowConsecutiveSendFails);

  g_espnowHealthy = EspNow_init(DONGLE_MAC);
  EspNow_setCommandCallback(handleCommandLine);

  if (g_espnowHealthy) {
    g_espnowConsecutiveSendFails = 0;
    Serial.println("[ESP-NOW] reinit OK");
  } else {
    Serial.println("[ESP-NOW] reinit FAILED");
  }

  return g_espnowHealthy;
}

String buildDualImuCsv() {
  // Update both IMUs; collect their CSV into strings without trailing newline.
  imu1.update();
  imu2.update();

  StringStreamSink p1, p2;
  imu1.printCSV(p1);
  imu2.printCSV(p2);
  rstrip_nl(p1.s);
  rstrip_nl(p2.s);

  // epoch_ms is millis() here (not absolute Unix time)
  uint32_t ms = millis();

  String out;
  out.reserve(16 + p1.s.length() + p2.s.length() + 4);
  out += String(ms);
  out += ",";
  out += p1.s;
  out += ",";
  out += p2.s;
  out += "\n";
  return out;
}

// ── 100 Hz body control + telemetry task ─────────────────────────────────────
void EspNowTxTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  TickType_t next = xTaskGetTickCount();
  const float dt = 0.01f;

  for (;;) {
    vTaskDelayUntil(&next, period);

    String csv = buildDualImuCsv();

    // Telemetry verso ROS su USB seriale, decimata.
    // 100 Hz / 5 = 20 Hz con USB a 115200.
    if (usbTelemetryEnabled) {
  uint8_t div = usbTelemetryDiv;
  if (div < 1) div = 1;

  usbTelemCounter++;
  if (usbTelemCounter >= div) {
    usbTelemCounter = 0;
    Serial.print(csv);
  }
}

    // Onboard attitude stabilization should live here (time-critical),
    // while high-level ROS only sends bias wrench commands.
    float q[4];
    getAttitudeQuat(q);
    const float pitchRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    const float yawRad   = yawFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    const float pitchDeg = pitchRad * 180.0f / 3.1415926f;

    bool assistActive = assistEnabled && ((int32_t)(millis() - assistEndMs) < 0);
    if (assistEnabled && !assistActive) assistEnabled = false;

    const bool manualActive = isManualThrusterActive();

    if (manualThrusterMode && !manualActive) {
      manualThrusterMode = false;
      stopAllThrusters();
    }

    if (manualActive) {
      // Manual bench commands THR / t1..t6 / pth own the PWM outputs until timeout.
      lastPitchCmd = 0.0f;
      lastYawCmd = 0.0f;

    } else {
      // Pitch hold on dedicated pair T5/T6
      if (pitchCtlEnabled || assistActive) {
        if (fabsf(pitchDeg) > pitchSafeDeg) {
          lastPitchCmd = 0.0f;
          stopPitchThrusters();

        } else if (assistActive) {
          const float e = pitchRefRad - pitchRad;
          const float D = assistKd * (e - pitchPrevE) / dt;
          pitchPrevE = e;
          lastPitchCmd = clampf(assistKp * e + D, -assistUmax, assistUmax);
          setPitchThrusters(lastPitchCmd);

        } else {
          lastPitchCmd = pitchPidStep(pitchRad, dt);
          setPitchThrusters(PITCH_PID_OUTPUT_SIGN * lastPitchCmd);
        }

      } else {
        lastPitchCmd = 0.0f;
        stopPitchThrusters();
      }

      // Yaw hold runs on the four lateral thrusters and adds to the high-level open-loop cmdMz.
      float yawHoldMz = 0.0f;
        if (yawCtlEnabled) {
        yawHoldMz = YAW_PID_OUTPUT_SIGN * yawPidStep(yawRad, dt);
      }

      lastYawCmd = yawHoldMz;

      const bool lateralActive = propCtrlEnabled || yawCtlEnabled;
      if (lateralActive) {
          setLateralThrustersFromWrench(cmdFx, cmdFy, cmdMz + yawHoldMz);
      } else {
        stopLateralThrusters();
      }
    }

    // ESP-NOW spento per test ROS via USB seriale.
    if (espnowTelemetryEnabled) {
      EspNow_send(csv);
     }
  }
}