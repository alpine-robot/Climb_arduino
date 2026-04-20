#pragma once
#include <Arduino.h>

class Movella {
public:
  // Construct with a HardwareSerial port and an optional sensor ID you choose
  explicit Movella(HardwareSerial& port, int id = 0);

  // Configure UART and (optionally) print CSV header to a Stream (e.g., Serial)
  bool begin(uint32_t baud = 115200, int8_t rxPin = -1, int8_t txPin = -1, Stream* headerOut = nullptr);

  // Non-blocking parse; returns true when a *full* packet with all fields is decoded
  bool update();

  // Accessors (copy out)
  void getQuaternion(float out[4]) const;
  void getAcceleration(float out[3]) const;
  void getGyro(float out[3]) const;

  // Convenience: print one CSV line (q0..q3, ax..az, gx..gz, id)
  void printCSV(Stream& out) const;

  // Measured packet frequency (Hz), updated every ~1s
  float frequencyHz() const { return freq_; }

  // Your own tag for the source (e.g., which IMU this is)
  int id() const { return id_; }

  uint32_t goodFrames() const { return goodFrames_; }
  uint32_t badChecksumFrames() const { return badCkFrames_; }

private:
  // Internal helpers
  bool readXbusPacket();
  bool parseQuaternion(const uint8_t* buf, int len, float q[4]);
  bool parseAcceleration(const uint8_t* buf, int len, float acc[3]);
  bool parseGyro(const uint8_t* buf, int len, float gyro[3]);

  HardwareSerial& serial_;
  int id_;

  // Stream & packet state
  uint8_t  buf_[256];
  int      pktLen_     = 0;
  int      idx_        = 0;
  bool     inPacket_   = false;

  // Latest decoded values
  bool     haveQ_      = false;
  bool     haveA_      = false;
  bool     haveG_      = false;
  float    q_[4]       = {0,0,0,1};
  float    acc_[3]     = {0,0,0};
  float    gyro_[3]    = {0,0,0};

  // Frequency tracking
  unsigned long lastTickMs_ = 0;
  int           counter_    = 0;
  float         freq_       = 0.f;

  uint32_t goodFrames_ = 0;
  uint32_t badCkFrames_ = 0;
};