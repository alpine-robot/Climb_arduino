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
#include <ESP32Servo.h>
#include "ServoValve.h"
#include "Motor.h"
#include "EscThruster.h"
#include "Movella.h"
#include "EspNow.h"
#include <esp_mac.h>  // at top, with other includes

// ── Peer (of dongle) MAC address — CHANGE THIS ───────────────────────────────────
uint8_t DONGLE_MAC[6] = { 0x50, 0x78, 0x7D, 0x16, 0xA9, 0x0C };

// ── Pins (adjust if needed) ───────────────────────────────────────────────────
static constexpr uint8_t SERVO1_PIN = 35;
static constexpr uint8_t SERVO2_PIN = 36;
static constexpr uint8_t RPWM_PIN   = 37;   // BTS7960 RPWM
static constexpr uint8_t LPWM_PIN   = 38;   // BTS7960 LPWM

static constexpr uint8_t PIN_T1 = 6;
static constexpr uint8_t PIN_T2 = 7;
static constexpr uint8_t PIN_T3 = 8;
static constexpr uint8_t PIN_T4 = 9;

static constexpr uint8_t PIN_T5 = 10;   // top pitch thruster
static constexpr uint8_t PIN_T6 = 11;   // bottom pitch thruster

// ── Objects ───────────────────────────────────────────────────────────────────
ServoValve ServoValve1(SERVO1_PIN);
ServoValve ServoValve2(SERVO2_PIN);

EscThruster  thr1(PIN_T1);
EscThruster  thr2(PIN_T2);
EscThruster  thr3(PIN_T3);
EscThruster  thr4(PIN_T4);

EscThruster  thr5(PIN_T5);   // upper pitch
EscThruster  thr6(PIN_T6);   // lower pitch

// Software-PWM Motor (keep low-moderate due to blocking servo frames)
Motor motor(RPWM_PIN, LPWM_PIN, /*pwmHz*/1000);

// Movella IMUs on two UARTs
HardwareSerial Xsens1(2);  // UART2: RX=16, TX=17
HardwareSerial Xsens2(1);  // UART1: RX=5,  TX=4
Movella imu1(Xsens1, 1);
Movella imu2(Xsens2, 2);

// ── Thruster timeout ─────────────────
volatile uint32_t lastThrCmdMs = 0;
static constexpr uint32_t THR_TIMEOUT_MS = 500; //300

volatile float cmdFx = 0.0f;   // body-frame force x
volatile float cmdFy = 0.0f;   // body-frame force y
volatile float cmdMz = 0.0f;   // body-frame yaw moment

volatile bool propCtrlEnabled = true;   // laterali da comando ROS/ESP-NOW

static constexpr float FX_GAIN = 1.0f;
static constexpr float FY_GAIN = 1.0f;
static constexpr float MZ_GAIN = 1.0f;

static constexpr float THR_MAX = 1.0f;


//assist mod full duty ------------------------------------------
volatile bool assistEnabled = false;
volatile uint32_t assistEndMs = 0;

volatile float assistUmax = 1.0f;     // più aggressivo dell'umax normale
volatile float assistKp = 3.0f;
volatile float assistKd = 0.25f;


// ── Pitch control ────────────────────────────────────────────────
volatile bool pitchCtlEnabled = false;
volatile float pitchRefRad = 0.0f;

volatile float Kp = 1.2f;
volatile float Ki = 0.0f;
volatile float Kd = 0.05f;

volatile float uMax = 0.6f;
volatile float pitchSafeDeg = 45.0f;

static float pid_I = 0.0f;
static float pid_prev_e = 0.0f;


// ── Simple line reader for Serial ─────────────────────────────────────────────
String line;
bool readLine(String& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out.trim(); return true; }
    out += c;
  }
  return false;
}

void printHelp() {
  Serial.println(F(
    "Commands:\n"
    "  s1 <deg>                 - set valve1 angle (0..90)\n"
    "  s2 <deg>                 - set valve2 angle (0..90)\n"
    "  THR,t1,t2,t3,t4,t5,t6    - set all thrusters duty\n"
    "  pth <val>                - manual pitch thrusters command in [-1..1]\n"
    "  pitch                    - print current pitch angle\n"
    "  apon                     - enable auto pitch control with pitch thrusters\n"
    "  apoff                    - disable auto pitch control\n"
    "  apzero                   - set current pitch as reference\n"
    "  pid <Kp> <Ki> <Kd>       - set pitch controller gains\n"
    "  umax <0..1>              - set max pitch thruster command\n"
    "  m<val>                   - motor command in [-1..1]\n"
    "  mf <hz>                  - set motor PWM frequency\n"
    "  mstop                    - stop motor\n"
    "  WRC,fx,fy,mz            - set lateral wrench command\n"
    "  pron                    - enable lateral prop control\n"
    "  proff                   - disable lateral prop control\n"
    "  ext                      - test motor extend\n"
    "  ret                      - test motor retract\n"
    "  status                   - print current state\n"
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





static inline float clampf(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }

// Pitch (rad) from quaternion (assume q0=w, q1=x, q2=y, q3=z)
static float pitchFromQuatWXYZ(float w, float x, float y, float z) {
  float sinp = 2.0f * (w*y - z*x);
  sinp = clampf(sinp, -1.0f, 1.0f);
  return asinf(sinp);
}

static float pitchPidStep(float pitchRad, float dt) {
  float e = pitchRefRad - pitchRad;

  // deadband 0.5°
  const float dead = 0.5f * (3.1415926f / 180.0f);
  if (fabsf(e) < dead) e = 0.0f;

  float P = Kp * e;

  pid_I += Ki * e * dt;
  pid_I = clampf(pid_I, -0.5f, 0.5f);

  float D = Kd * (e - pid_prev_e) / dt;
  pid_prev_e = e;

  float u = P + pid_I + D;

  float u_sat = clampf(u, -uMax, uMax);
  if (u != u_sat) pid_I *= 0.95f; // anti-windup morbido
  return u_sat;
}


// opzionale: rampetta per non dare step bruschi agli ESC
static float pitchTopCmd = 0.0f;
static float pitchBotCmd = 0.0f;

static inline float rampTowards(float current, float target, float step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}



static inline void setPitchThrusters(float pitchCmd) {
  pitchCmd = clampf(pitchCmd, -1.0f, 1.0f);

  float t5_target = 0.0f;
  float t6_target = 0.0f;

  if (pitchCmd > 0.0f) {
    t5_target = pitchCmd;
  } else if (pitchCmd < 0.0f) {
    t6_target = -pitchCmd;
  }

  pitchTopCmd = rampTowards(pitchTopCmd, t5_target, 0.03f);
  pitchBotCmd = rampTowards(pitchBotCmd, t6_target, 0.03f);

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

  // yaw: right = T1 + T3, left = T2 + T4
  if (mz > 0.0f) {
    t1 += MZ_GAIN * mz;
    t3 += MZ_GAIN * mz;
  } else if (mz < 0.0f) {
    t2 += MZ_GAIN * (-mz);
    t4 += MZ_GAIN * (-mz);
  }

  // lateral translation:
  // right = T2 + T3
  // left  = T1 + T4
  if (fy > 0.0f) {
    t2 += FY_GAIN * fy;
    t3 += FY_GAIN * fy;
  } else if (fy < 0.0f) {
    t1 += FY_GAIN * (-fy);
    t4 += FY_GAIN * (-fy);
  }

  // eventuale Fx contro il muro:
  // scegli il pattern vero del tuo frame.
  // Per ora esempio: stesso pattern del wall push che usavi
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

  thr1.setThrottle(t1);
  thr2.setThrottle(t2);
  thr3.setThrottle(t3);
  thr4.setThrottle(t4);
}

static inline void stopLateralThrusters() {
  thr1.stop();
  thr2.stop();
  thr3.stop();
  thr4.stop();
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


// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);
  
  lastThrCmdMs = millis();

  // --- Valves ---
  ServoValve1.begin();
  ServoValve2.begin();
  
  thr1.begin();
  thr2.begin();
  thr3.begin();
  thr4.begin();
  thr5.begin();
  thr6.begin();

   // arm ESC a zero
  stopAllThrusters();
  delay(3000);

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

  // ⚠️ Initialize ESP-NOW and set the command callback BEFORE starting the TX task
  bool espnow_ok = EspNow_init(DONGLE_MAC);
  Serial.printf(
    "[ESP-NOW] local=%02X:%02X:%02X:%02X:%02X:%02X  "
    "peer=%02X:%02X:%02X:%02X:%02X:%02X  init_ok=%s\n",
    localMac[0],localMac[1],localMac[2],localMac[3],localMac[4],localMac[5],
    DONGLE_MAC[0],DONGLE_MAC[1],DONGLE_MAC[2],DONGLE_MAC[3],DONGLE_MAC[4],DONGLE_MAC[5],
    espnow_ok ? "true" : "false"
  );

  if (espnow_ok) {
    EspNow_setCommandCallback(handleCommandLine);

    // Launch 100 Hz telemetry TX task ONLY if init succeeded
    BaseType_t ok = xTaskCreatePinnedToCore(
        EspNowTxTask, "espnow_tx", 4096, nullptr, 2, nullptr, APP_CPU_NUM);
    if (ok != pdPASS) Serial.println("[ESP-NOW] ERROR: TX task create failed!");
  } else {
    Serial.println("[ESP-NOW] init failed — TX task not started.");
  }

  Serial.println("Ready.");
  printHelp();
  Serial.print("> ");
}
// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  // Parse Serial commands
  if (readLine(line)) {
    String cmd = line; 
    cmd.trim();
    handleCommandLine(cmd);  // shared parser
    line = "";
    Serial.print("> ");
  }

  // Pump ESP-NOW RX queue → dispatches to handleCommandLine()
  EspNow_loop();

// Safety timeout for thrusters
if (millis() - lastThrCmdMs > THR_TIMEOUT_MS) {
  cmdFx = 0.0f;
  cmdFy = 0.0f;
  cmdMz = 0.0f;
  stopLateralThrusters();

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

  if (low.startsWith("s1")) {
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
  propCtrlEnabled = false;

  thr1.setThrottle(t1);
  thr2.setThrottle(t2);
  thr3.setThrottle(t3);
  thr4.setThrottle(t4);

  if (!pitchCtlEnabled) {
    thr5.setThrottle(t5);
    thr6.setThrottle(t6);
  }

  lastThrCmdMs = millis();
  Serial.printf("THR -> %.2f %.2f %.2f %.2f %.2f %.2f\n", t1, t2, t3, t4, t5, t6);
}else {
      Serial.println("ERR: usage THR,t1,t2,t3,t4,t5,t6");
    }

  } else if (low.startsWith("pth ")) {
  float u = 0.0f;
  int n = sscanf(cmd.c_str(), "pth %f", &u);
  if (n == 1) {
    pitchCtlEnabled = false;
    assistEnabled = false;
    setPitchThrusters(u);
    lastThrCmdMs = millis();
    Serial.printf("Pitch thrusters -> %.2f\n", u);
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
    imu1.getQuaternion(q);
    float pitchRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    float pitchDeg = pitchRad * 180.0f / 3.1415926f;
    Serial.printf("Pitch = %.2f deg\n", pitchDeg);

  } else if (low == "apon") {
    pitchCtlEnabled = true;
    pid_I = 0.0f;
    pid_prev_e = 0.0f;
    Serial.println("AutoPitch ON (pitch thrusters)");

  } else if (low == "apoff") {
  pitchCtlEnabled = false;
  assistEnabled = false;
  stopPitchThrusters();
  motor.stop();
  Serial.println("AutoPitch OFF");

  } else if (low == "pron") {
  propCtrlEnabled = true;
  Serial.println("Lateral prop control ON");
} else if (low == "proff") {
  propCtrlEnabled = false;
  cmdFx = 0.0f;
  cmdFy = 0.0f;
  cmdMz = 0.0f;
  stopLateralThrusters();
  Serial.println("Lateral prop control OFF");
  }else if (low == "apzero") {
    float q[4];
    imu1.getQuaternion(q);
    pitchRefRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
    pid_I = 0.0f;
    pid_prev_e = 0.0f;
    Serial.println("AutoPitch reference set (apzero)");

  } else if (low.startsWith("pid ")) {
    float p, i, d;
    int n = sscanf(cmd.c_str(), "pid %f %f %f", &p, &i, &d);
    if (n == 3) {
      Kp = p;
      Ki = i;
      Kd = d;
      Serial.printf("PID set: Kp=%.3f Ki=%.3f Kd=%.3f\n", Kp, Ki, Kd);
    } else {
      Serial.println("Usage: pid <Kp> <Ki> <Kd>");
    }

  } else if (low.startsWith("umax ")) {
    float x;
    int n = sscanf(cmd.c_str(), "umax %f", &x);
    if (n == 1) {
      uMax = clampf(x, 0.0f, 1.0f);
      Serial.printf("uMax=%.2f\n", uMax);
    } else {
      Serial.println("Usage: umax <0..1>");
    }

  } else if (low.startsWith("assist ")) {
    int ms = 0;
    int n = sscanf(cmd.c_str(), "assist %d", &ms);
    if (n == 1) {
      if (ms < 50) ms = 50;
      if (ms > 2000) ms = 2000;
      assistEnabled = true;
      assistEndMs = millis() + (uint32_t)ms;
      pid_I = 0.0f;
      pid_prev_e = 0.0f;
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
  Serial.printf("cmdFx=%.2f cmdFy=%.2f cmdMz=%.2f\n", cmdFx, cmdFy, cmdMz);
  Serial.printf("T1=%.2f T2=%.2f T3=%.2f T4=%.2f T5=%.2f T6=%.2f\n",
    thr1.lastThrottle(), thr2.lastThrottle(), thr3.lastThrottle(),
    thr4.lastThrottle(), thr5.lastThrottle(), thr6.lastThrottle());
    
  Serial.printf("ESP-NOW tx_count: %lu\n", (unsigned long)EspNow_txCount());
} else if (low == "help" || low == "?") {
    printHelp();

  } else if (low.length() > 0) {
    Serial.println("Unknown. Type 'help'.");
  }
}

// ── Build dual-IMU CSV line ───────────────────────────────────────────────────
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

// ── 100 Hz ESP-NOW TX task ───────────────────────────────────────────────────
void EspNowTxTask(void* arg) {
  (void)arg;
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  TickType_t next = xTaskGetTickCount();
  const float dt = 0.01f;
  for (;;) {
    vTaskDelayUntil(&next, period);

    String csv = buildDualImuCsv();
    // Optional: echo to USB for debug
    // Serial.print(csv);

   // pitch control (pitch thrusters)
 bool assistActive = assistEnabled && ((int32_t)(millis() - assistEndMs) < 0);
if (assistEnabled && !assistActive) assistEnabled = false;

if (pitchCtlEnabled || assistActive) {
  float q[4];
  imu1.getQuaternion(q);
  float pitchRad = pitchFromQuatWXYZ(q[0], q[1], q[2], q[3]);
  float pitchDeg = pitchRad * 180.0f / 3.1415926f;

  if (fabsf(pitchDeg) > pitchSafeDeg) {
    stopPitchThrusters();
  } else {
    float kp = assistActive ? assistKp : Kp;
    float kd = assistActive ? assistKd : Kd;
    float umax = assistActive ? assistUmax : uMax;

    float e = pitchRefRad - pitchRad;
    float D = kd * (e - pid_prev_e) / 0.01f;
    pid_prev_e = e;

    float u = kp * e + D;
    u = clampf(u, -umax, umax);

    setPitchThrusters(u);
  }
} else {
  stopPitchThrusters();
}
if (propCtrlEnabled) {
  setLateralThrustersFromWrench(cmdFx, cmdFy, cmdMz);
}
    // Send over ESP-NOW (silently ignore if not initialized)
    EspNow_send(csv);
  }
}