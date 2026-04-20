#include "Dongle.h"

DongleController* DongleController::instance = nullptr;

enum CommState { SEND_SX, WAIT_SX, SEND_DX, WAIT_DX };
CommState state = SEND_SX;

unsigned long lastSend = 0;
const unsigned long TIMEOUT_MS = 50;

DongleController::DongleController(const uint8_t* mac_dx, const uint8_t* mac_sx) {
  memcpy(macDx, mac_dx, 6);
  memcpy(macSx, mac_sx, 6);
  instance = this;

  strncpy(lastCmdDx, "dx:closed_loop", MESSAGE_SIZE);
  strncpy(lastCmdSx, "sx:closed_loop", MESSAGE_SIZE);
}


void DongleController::begin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSentStatic);
  esp_now_register_recv_cb(onDataRecvStatic);

  // Add DX peer
  memcpy(peerInfoDx.peer_addr, macDx, 6);
  peerInfoDx.channel = 0;
  peerInfoDx.encrypt = false;
  esp_now_add_peer(&peerInfoDx);

  // Add SX peer
  memcpy(peerInfoSx.peer_addr, macSx, 6);
  peerInfoSx.channel = 0;
  peerInfoSx.encrypt = false;
  esp_now_add_peer(&peerInfoSx);

  Serial.print("🔧 Dongle MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("✅ Starting round-robin communication");
}

void DongleController::loop() {
  unsigned long now = millis();
  handleSerialInput();  // call first

  if (state == SEND_SX) {
    prepareAndSend(lastCmdSx, macSx);
    state = WAIT_SX;
    lastSend = now;
  } else if (state == SEND_DX) {
    prepareAndSend(lastCmdDx, macDx);
    state = WAIT_DX;
    lastSend = now;
  } else if ((state == WAIT_SX || state == WAIT_DX) && now - lastSend > TIMEOUT_MS) {
    Serial.println("Timeout — skipping to next...");
    state = (state == WAIT_SX) ? SEND_DX : SEND_SX;
  }
}


void DongleController::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&recvMsg, incomingData, sizeof(recvMsg));
  recvMsg.text[MESSAGE_SIZE - 1] = '\0';  // Safety

  const char* sender = "❓ Unknown";
  if (memcmp(mac, macDx, 6) == 0) {
    sender = "📟 Arganello DX";
  } else if (memcmp(mac, macSx, 6) == 0) {
    sender = "📟 Arganello SX";
  }

  Serial.print("📨 From ");
  Serial.print(sender);
  Serial.print(": ");
  Serial.println(recvMsg.text);

  // Move to next target
  state = (state == WAIT_SX) ? SEND_DX : SEND_SX;
}

void DongleController::handleSerialInput() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (!(input.startsWith("dx:") || input.startsWith("sx:"))) {
    Serial.println("⚠️ Invalid. Use 'dx:' or 'sx:' prefix.");
    return;
  }

  // Pad input to 20 characters
  char padded[MESSAGE_SIZE + 1];
  memset(padded, ' ', MESSAGE_SIZE);
  strncpy(padded, input.c_str(), MESSAGE_SIZE);
  padded[MESSAGE_SIZE] = '\0';

  // Store and send
  if (input.startsWith("dx:")) {
    strncpy(lastCmdDx, padded, MESSAGE_SIZE);
    prepareAndSend(lastCmdDx, macDx);
    state = WAIT_DX;
  } else {
    strncpy(lastCmdSx, padded, MESSAGE_SIZE);
    prepareAndSend(lastCmdSx, macSx);
    state = WAIT_SX;
  }

  lastSend = millis();
}


void DongleController::prepareAndSend(const char* cmd, const uint8_t* mac) {
  memset(sendMsg.text, ' ', MESSAGE_SIZE);
  strncpy(sendMsg.text, cmd, MESSAGE_SIZE);
  esp_now_send(mac, (uint8_t*)&sendMsg, sizeof(sendMsg));

  Serial.print("📤 Sent: ");
  Serial.write(sendMsg.text, MESSAGE_SIZE);
  Serial.println();
}

void DongleController::onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (instance) instance->onDataSent(mac_addr, status);
}

void DongleController::onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (instance) instance->onDataRecv(mac, incomingData, len);
}

void DongleController::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("📤 Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "✅ OK" : "❌ Failed");
}