#include "EspNow.h"
#include <string.h>

static esp_now_peer_info_t peer{};
static EspNowCommandCallback g_cb = nullptr;
static QueueHandle_t g_queue = nullptr;

static volatile uint32_t g_txCount = 0;
static volatile uint32_t g_rxCount = 0;
static volatile uint32_t g_rxDroppedCount = 0;
static volatile uint32_t g_rxTruncatedCount = 0;
static volatile uint32_t g_sendFailCount = 0;

static constexpr size_t ESPNOW_LINE_MAX  = 256;
static constexpr size_t ESPNOW_QUEUE_LEN = 24;

struct EspNowRxMsg {
  uint16_t len;
  char text[ESPNOW_LINE_MAX];
};

static void trimInPlace(char* s) {
  if (!s) return;

  char* p = s;
  while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
  if (p != s) memmove(s, p, strlen(p) + 1);

  int n = (int)strlen(s);
  while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t' || s[n - 1] == '\r' || s[n - 1] == '\n')) {
    s[n - 1] = '\0';
    --n;
  }
}

static void pushRxMsg(const uint8_t* data, int len) {
  if (!g_queue || !data || len <= 0) return;

  EspNowRxMsg msg{};
  int n = len;

  if (n >= (int)ESPNOW_LINE_MAX) {
    n = (int)ESPNOW_LINE_MAX - 1;
    g_rxTruncatedCount++;
  }

  memcpy(msg.text, data, n);
  msg.text[n] = '\0';
  trimInPlace(msg.text);
  msg.len = (uint16_t)strlen(msg.text);

  if (msg.len == 0) return;

  g_rxCount++;

  // Keep newest commands if saturated
  if (xQueueSend(g_queue, &msg, 0) != pdTRUE) {
    EspNowRxMsg dummy{};
    xQueueReceive(g_queue, &dummy, 0);  // drop oldest
    if (xQueueSend(g_queue, &msg, 0) != pdTRUE) {
      g_rxDroppedCount++;
    }
  }
}

#if ESP_IDF_VERSION_MAJOR >= 5
static void onRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  (void)info;
  pushRxMsg(data, len);
}
#else
static void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  pushRxMsg(data, len);
}
#endif

#if ESP_IDF_VERSION_MAJOR >= 5
static void onSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  (void)tx_info;
  if (status == ESP_NOW_SEND_SUCCESS) g_txCount++;
  else g_sendFailCount++;
}
#else
static void onSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
  if (status == ESP_NOW_SEND_SUCCESS) g_txCount++;
  else g_sendFailCount++;
}
#endif

bool EspNow_init(const uint8_t peer_mac[6]) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);   // important for latency/jitter
  WiFi.disconnect();

  esp_now_deinit();       // safe if re-init happens

  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peer_mac, 6);
  peer.channel = 0;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;

  esp_now_del_peer(peer.peer_addr); // ignore error if not present
  if (esp_now_add_peer(&peer) != ESP_OK) return false;

  if (g_queue) {
    vQueueDelete(g_queue);
    g_queue = nullptr;
  }

  g_queue = xQueueCreate(ESPNOW_QUEUE_LEN, sizeof(EspNowRxMsg));
  return (g_queue != nullptr);
}

bool EspNow_send(const String& line) {
  if (!line.length()) return true;

  esp_err_t err = esp_now_send(
    peer.peer_addr,
    reinterpret_cast<const uint8_t*>(line.c_str()),
    line.length()
  );

  if (err != ESP_OK) {
    g_sendFailCount++;
    return false;
  }
  return true;
}

void EspNow_loop() {
  if (!g_queue || !g_cb) return;

  EspNowRxMsg msg{};
  while (xQueueReceive(g_queue, &msg, 0) == pdTRUE) {
    String s(msg.text);   // build String outside WiFi callback
    s.trim();
    if (s.length()) {
      g_cb(s);
    }
  }
}

void EspNow_setCommandCallback(EspNowCommandCallback cb) {
  g_cb = cb;
}

uint32_t EspNow_txCount() { return g_txCount; }
uint32_t EspNow_rxCount() { return g_rxCount; }
uint32_t EspNow_rxDroppedCount() { return g_rxDroppedCount; }
uint32_t EspNow_rxTruncatedCount() { return g_rxTruncatedCount; }
uint32_t EspNow_sendFailCount() { return g_sendFailCount; }