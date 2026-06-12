// DONGLE ESP-NOW <-> USB bridge (patched for ROS2)
// Key fixes:
// 1) USB serial outputs ONLY CSV telemetry lines by default (no [RX ...] prefixes)
// 2) automatic ESP-NOW re-init if no packets arrive for a while
// 3) optional debug logs guarded behind DEBUG_SERIAL
// 4) keeps immediate command TX + 100 Hz resend fallback

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>

uint8_t ONBOARD_MAC[6] = { 0xCC, 0xBA, 0x97, 0x14, 0x0A, 0x14 };

static constexpr bool DEBUG_SERIAL = false;   // set true only for manual debug, false for ROS2 CSV parsing
static constexpr uint32_t RX_STILL_OK_MS = 50;
static constexpr uint32_t REINIT_IF_SILENT_MS = 1500;
static constexpr uint32_t REINIT_COOLDOWN_MS = 500;

String line;
static volatile uint32_t g_rxCount = 0;
static volatile TickType_t g_lastRxTick = 0;
static String g_lastCmd;
static TickType_t g_lastCmdTick = 0;
static uint32_t g_lastReinitMs = 0;

static inline void dbg(const char* s) {
  if (DEBUG_SERIAL) Serial.println(s);
}

bool readLine(String& out) {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out.trim(); return true; }
    out += c;
  }
  return false;
}

bool addPeer() {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, ONBOARD_MAC, 6);
  peer.channel = 0;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_del_peer(ONBOARD_MAC);
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect();

  esp_now_deinit();
  if (esp_now_init() != ESP_OK) {
    dbg("[ESP-NOW] init FAILED");
    return false;
  }

  esp_now_register_recv_cb([](const esp_now_recv_info* info, const uint8_t* data, int len) {
    (void)info;
    g_rxCount++;
    g_lastRxTick = xTaskGetTickCount();
    if (data && len > 0) {
      // IMPORTANT: ROS2 parser expects plain CSV lines, no prefixes
      Serial.write(data, len);
      if (data[len - 1] != '\n') Serial.println();
    }
  });

  esp_now_register_send_cb([](const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    (void)tx_info;
    (void)status;
  });

  if (!addPeer()) {
    dbg("[ESP-NOW] add_peer FAILED");
    return false;
  }

  return true;
}

void maybeReinitEspNow() {
  uint32_t nowMs = millis();
  uint32_t lastRxMs = (uint32_t)(g_lastRxTick * portTICK_PERIOD_MS);
  bool silentTooLong = (nowMs > lastRxMs) && ((nowMs - lastRxMs) > REINIT_IF_SILENT_MS);
  bool cooledDown = (nowMs - g_lastReinitMs) > REINIT_COOLDOWN_MS;

  if (silentTooLong && cooledDown) {
    g_lastReinitMs = nowMs;
    dbg("[ESP-NOW] silent too long -> reinit");
    initEspNow();
    // refresh lastRxTick so we do not re-enter immediately if robot is currently off
    g_lastRxTick = xTaskGetTickCount();
  }
}

void ResendTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(10);
  TickType_t next = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&next, period);

    TickType_t now = xTaskGetTickCount();
    bool recentlyReceiving = (now - g_lastRxTick) <= pdMS_TO_TICKS(RX_STILL_OK_MS);

    if (!recentlyReceiving && g_lastCmd.length() > 0) {
      esp_now_send(ONBOARD_MAC, (const uint8_t*)g_lastCmd.c_str(), g_lastCmd.length());
    }
  }
}

void setup() {
  Serial.begin(1000000);
  delay(300);

  uint8_t myMac[6];
  esp_read_mac(myMac, ESP_MAC_WIFI_STA);
  if (DEBUG_SERIAL) {
    Serial.printf("[DONGLE] local=%02X:%02X:%02X:%02X:%02X:%02X peer=%02X:%02X:%02X:%02X:%02X:%02X\n",
      myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5],
      ONBOARD_MAC[0], ONBOARD_MAC[1], ONBOARD_MAC[2], ONBOARD_MAC[3], ONBOARD_MAC[4], ONBOARD_MAC[5]);
  }

  initEspNow();
  g_lastRxTick = xTaskGetTickCount();
  xTaskCreatePinnedToCore(ResendTask, "resend_100hz", 4096, nullptr, 2, nullptr, APP_CPU_NUM);
}

void loop() {
  if (readLine(line)) {
    String cmd = line;
    cmd.trim();
    if (cmd.length()) {
      esp_now_send(ONBOARD_MAC, (const uint8_t*)cmd.c_str(), cmd.length());
      g_lastCmd = cmd;
      g_lastCmdTick = xTaskGetTickCount();
    }
    line = "";
  }

  maybeReinitEspNow();
}
