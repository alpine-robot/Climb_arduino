#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>

// === Public API ===
bool EspNow_init(const uint8_t peer_mac[6]);
bool EspNow_send(const String& line);
void EspNow_loop();

// Command callback
typedef void (*EspNowCommandCallback)(const String& cmd);
void EspNow_setCommandCallback(EspNowCommandCallback cb);

// Stats / debug
uint32_t EspNow_txCount();
uint32_t EspNow_rxCount();
uint32_t EspNow_rxDroppedCount();
uint32_t EspNow_rxTruncatedCount();
uint32_t EspNow_sendFailCount();