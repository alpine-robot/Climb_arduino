#include "Movella.h"

// Xbus markers/sizes
static constexpr uint8_t  PREAMBLE = 0xFA;
static constexpr uint8_t  BID      = 0xFF; // Bus ID for MTData2 frames

Movella::Movella(HardwareSerial& port, int id)
: serial_(port), id_(id) {}

bool Movella::begin(uint32_t baud, int8_t rxPin, int8_t txPin, Stream* headerOut) {
  if (rxPin >= 0 && txPin >= 0) {
    serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
  } else {
    serial_.begin(baud);
  }
  // Optional header for CSV logs
  if (headerOut) {
    headerOut->println(F("q0\tq1\tq2\tq3\tax\tay\taz\tgx\tgy\tgz\tid"));
  }
  lastTickMs_ = millis();
  return true;
}

bool Movella::update() {
  // Try to assemble one full packet
  if (!readXbusPacket()) return false;

  // Reset decoded flags for this packet
  haveQ_ = parseQuaternion(buf_, pktLen_, q_);
  haveA_ = parseAcceleration(buf_, pktLen_, acc_);
  haveG_ = parseGyro(buf_, pktLen_, gyro_);

  // Only signal "true" when we got all 3 blocks
  bool all = haveQ_ && haveA_ && haveG_;
  if (all) {
    // frequency update (per complete packet)
    ++counter_;
    unsigned long now = millis();
    unsigned long dt = now - lastTickMs_;
    if (dt >= 1000) {
      freq_ = counter_ * 1000.0f / float(dt);
      counter_ = 0;
      lastTickMs_ = now;
    }
  }
  return all;
}

void Movella::getQuaternion(float out[4]) const { for (int i=0;i<4;++i) out[i]=q_[i]; }
void Movella::getAcceleration(float out[3]) const { for (int i=0;i<3;++i) out[i]=acc_[i]; }
void Movella::getGyro(float out[3]) const { for (int i=0;i<3;++i) out[i]=gyro_[i]; }

void Movella::printCSV(Stream& out) const {
  out.print(q_[0], 6); out.print('\t');
  out.print(q_[1], 6); out.print('\t');
  out.print(q_[2], 6); out.print('\t');
  out.print(q_[3], 6); out.print('\t');
  out.print(acc_[0], 6); out.print('\t');
  out.print(acc_[1], 6); out.print('\t');
  out.print(acc_[2], 6); out.print('\t');
  out.print(gyro_[0], 6); out.print('\t');
  out.print(gyro_[1], 6); out.print('\t');
  out.print(gyro_[2], 6); out.print('\t');
  out.println(id_);
}

// ---------- private ----------

// Xbus checksum: 1 byte, two's complement, so that:
// (sum of bytes from BID to end of PAYLOAD + CHECKSUM) mod 256 == 0
static bool verifyXbusChecksum(const uint8_t* frame, int payloadLen) {
  // frame layout (when assembled in readXbusPacket):
  // [0]=PREAMBLE, [1]=BID, [2]=MID, [3]=LEN, [4..4+LEN-1]=PAYLOAD, [4+LEN]=CHECKSUM
  uint8_t sum = 0;
  for (int k = 1; k <= 3 + payloadLen; ++k) { // BID..LEN..PAYLOAD
    sum = uint8_t(sum + frame[k]);
  }
  sum = uint8_t(sum + frame[4 + payloadLen]); // checksum byte
  return (sum == 0);
}



bool Movella::readXbusPacket() {
  while (serial_.available()) {
    uint8_t b = serial_.read();

    if (!inPacket_) {
      if (b == PREAMBLE) {
        buf_[0]   = b;
        idx_      = 1;
        inPacket_ = true;
      }
      continue;
    }

    // In-packet
    if (idx_ < (int)sizeof(buf_)) {
      buf_[idx_++] = b;
    } else {
      // overflow: reset state
      inPacket_ = false;
      idx_ = 0;
      continue;
    }

    // Byte 1 must be BID (0xFF)
    if (idx_ == 2 && buf_[1] != BID) {
      inPacket_ = false;
      idx_ = 0;
      continue;
    }

    // Byte 3 (index=3) is LEN (payload length)
    if (idx_ == 4) {
      pktLen_ = buf_[3];
      if (pktLen_ < 0 || pktLen_ > 250) {
        inPacket_ = false;
        idx_ = 0;
        continue;
      }
    }

    // Frame size = preamble(1) + bid(1) + mid(1) + len(1) + payload(len) + checksum(1)
    // We've currently filled idx_ bytes; when idx_ == 5 + pktLen_ we have full frame.
    if (idx_ == pktLen_ + 5) {
      // Verify checksum BEFORE accepting
      if (!verifyXbusChecksum(buf_, pktLen_)) {
        // bad frame -> drop
        badCkFrames_++;
        inPacket_ = false;
        idx_ = 0;
        continue;
      }
      goodFrames_++;

      inPacket_ = false;
      int payloadStart = 4;      // buf_[4] .. buf_[4+pktLen_-1]
      // Shift payload to the beginning of buf_ for simpler parsing
      memmove(buf_, &buf_[payloadStart], pktLen_);
      idx_ = 0;
      // Now buf_[0..pktLen_-1] contains the payload only
      return true;
    }
  }
  return false;
}

// Parse blocks in the MTData2 payload.
// Quaternion block: 0x20 0x10 0x10 then 4x float32 big-endian
bool Movella::parseQuaternion(const uint8_t* buffer, int length, float q[4]) {
  for (int i = 0; i <= length - (3 + 16); ++i) {
    if (buffer[i] == 0x20 && buffer[i+1] == 0x10 && buffer[i+2] == 0x10) {
      for (int j = 0; j < 4; ++j) {
        uint32_t v = (uint32_t(buffer[i + 3 + j*4]) << 24) |
                     (uint32_t(buffer[i + 4 + j*4]) << 16) |
                     (uint32_t(buffer[i + 5 + j*4]) << 8)  |
                     (uint32_t(buffer[i + 6 + j*4]));
        float f;
        memcpy(&f, &v, sizeof(float));
        q[j] = f;
      }
      return true;
    }
  }
  return false;
}

// Acceleration block: 0x40 0x20 0x0C then 3x float32 big-endian
bool Movella::parseAcceleration(const uint8_t* buffer, int length, float acc[3]) {
  for (int i = 0; i <= length - (3 + 12); ++i) {
    if (buffer[i] == 0x40 && buffer[i+1] == 0x20 && buffer[i+2] == 0x0C) {
      for (int j = 0; j < 3; ++j) {
        uint32_t v = (uint32_t(buffer[i + 3 + j*4]) << 24) |
                     (uint32_t(buffer[i + 4 + j*4]) << 16) |
                     (uint32_t(buffer[i + 5 + j*4]) << 8)  |
                     (uint32_t(buffer[i + 6 + j*4]));
        float f;
        memcpy(&f, &v, sizeof(float));
        acc[j] = f;
      }
      return true;
    }
  }
  return false;
}

// Gyro block: 0x80 0x20 0x0C then 3x float32 big-endian
bool Movella::parseGyro(const uint8_t* buffer, int length, float gyro[3]) {
  for (int i = 0; i <= length - (3 + 12); ++i) {
    if (buffer[i] == 0x80 && buffer[i+1] == 0x20 && buffer[i+2] == 0x0C) {
      for (int j = 0; j < 3; ++j) {
        uint32_t v = (uint32_t(buffer[i + 3 + j*4]) << 24) |
                     (uint32_t(buffer[i + 4 + j*4]) << 16) |
                     (uint32_t(buffer[i + 5 + j*4]) << 8)  |
                     (uint32_t(buffer[i + 6 + j*4]));
        float f;
        memcpy(&f, &v, sizeof(float));
        gyro[j] = f;
      }
      return true;
    }
  }
  return false;
}




// ==============================
// Debug (optional) in climb_onboard_firmware.ino
// ==============================
// into EspNowTxTask() or loop() you can print every second:
//
//   static uint32_t t0 = 0;
//   if (millis() - t0 > 1000) {
//     t0 = millis();
//     Serial.printf("IMU1 good=%lu badCk=%lu\n", (unsigned long)imu1.goodFrames(), (unsigned long)imu1.badChecksumFrames());
//   }
//

