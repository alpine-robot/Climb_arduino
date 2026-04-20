// === DONGLE — ESP-NOW bridge with 100 Hz resend fallback =====================
// - IDF 5.x compatible callbacks
// - Read commands from Serial @115200 (e.g., "m0.2", "s1 45", "mf 200", "mstop")
// - Send command immediately, and keep re-sending it at 100 Hz **only when**
//   no ESP-NOW traffic has been received recently from the onboard peer.
// - Print any received payloads (CSV telemetry) to Serial.
//
// Replace ONBOARD_MAC with your onboard ESP32 MAC (STA).

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_mac.h>   // esp_read_mac()

// ── SET THIS to the onboard ESP32's MAC (peer) ───────────────────────────────
uint8_t ONBOARD_MAC[6] = { 0xCC, 0xBA, 0x97, 0x14, 0x0A, 0x14 }; // <-- CHANGE

// ── Serial line reader ───────────────────────────────────────────────────────
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

// ── State ────────────────────────────────────────────────────────────────────
static volatile uint32_t g_rxCount = 0;
static volatile TickType_t g_lastRxTick = 0;

static String g_lastCmd;              // last non-empty serial command
static TickType_t g_lastCmdTick = 0;  // when last command was submitted
static const TickType_t RX_STILL_OK_MS = 50; // consider "receiving" if RX within this window

// ── Callbacks (IDF 5.x signatures) ───────────────────────────────────────────
void onRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  g_rxCount++;
  g_lastRxTick = xTaskGetTickCount();

  // Print sender MAC and payload to Serial (telemetry CSV etc.)
  const uint8_t* mac = info ? info->src_addr : nullptr;
  if (mac) {
    Serial.printf("[RX %02X:%02X:%02X:%02X:%02X:%02X] ",
      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  } else {
    Serial.print("[RX] ");
  }
  if (data && len > 0) {
    Serial.write(data, len);
    if (data[len-1] != '\n') Serial.println();
  } else {
    Serial.println("(empty)");
  }
}

void onSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  // optional: debug TX status
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "[TX OK]" : "[TX FAIL]");
}

// ── 100 Hz task: resend last command only if no recent RX ────────────────────
void ResendTask(void* arg) {
  const TickType_t period = pdMS_TO_TICKS(10);  // 100 Hz
  TickType_t next = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&next, period);

    // If no recent RX from peer, repeat the last command (if any)
    TickType_t now = xTaskGetTickCount();
    bool recentlyReceiving = (now - g_lastRxTick) <= pdMS_TO_TICKS(RX_STILL_OK_MS);

    if (!recentlyReceiving && g_lastCmd.length() > 0) {
      esp_now_send(ONBOARD_MAC, (const uint8_t*)g_lastCmd.c_str(), g_lastCmd.length());
      // optional: Serial.printf("[REPEAT @100Hz] %s\n", g_lastCmd.c_str());
    }
  }
}

void setup() {
  Serial.begin(1000000);
  delay(1300);

  // WiFi / MAC
  WiFi.mode(WIFI_STA);
  uint8_t myMac[6]; esp_read_mac(myMac, ESP_MAC_WIFI_STA);
  Serial.printf("[DONGLE] local=%02X:%02X:%02X:%02X:%02X:%02X  peer=%02X:%02X:%02X:%02X:%02X:%02X\n",
    myMac[0],myMac[1],myMac[2],myMac[3],myMac[4],myMac[5],
    ONBOARD_MAC[0],ONBOARD_MAC[1],ONBOARD_MAC[2],ONBOARD_MAC[3],ONBOARD_MAC[4],ONBOARD_MAC[5]);

  // ESP-NOW init
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init FAILED");
    for(;;) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  // Add peer (onboard)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, ONBOARD_MAC, 6);
  peer.channel = 0;          // follow current STA channel
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] add_peer FAILED");
  } else {
    Serial.println("[ESP-NOW] ready.");
  }

  // Start 100 Hz resend task
  xTaskCreatePinnedToCore(ResendTask, "resend_100hz", 4096, nullptr, 2, nullptr, APP_CPU_NUM);

  Serial.println("Type commands like: m0.25  |  s1 45  |  s2 30  |  mf 200  |  mstop");
}

void loop() {
  // Read a full line from Serial; send immediately; remember it as last command
  if (readLine(line)) {
    String cmd = line; cmd.trim();
    if (cmd.length()) {
      esp_now_send(ONBOARD_MAC, (const uint8_t*)cmd.c_str(), cmd.length());
      g_lastCmd = cmd;
      g_lastCmdTick = xTaskGetTickCount();
      // Optional echo:
      // Serial.printf("[TX NOW] %s\n", cmd.c_str());
    }
    line = "";
  }

  // Nothing else here; RX is interrupt-driven, periodic repeat is in the task.
}