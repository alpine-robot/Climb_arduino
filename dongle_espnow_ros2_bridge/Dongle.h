#pragma once
#include <esp_now.h>
#include <WiFi.h>

#define MESSAGE_SIZE 20

typedef struct {
  char text[MESSAGE_SIZE];
} Message;

class DongleController {
public:
  DongleController(const uint8_t* mac_dx, const uint8_t* mac_sx);
  void begin();
  void loop();  // round-robin + serial input

private:
  Message sendMsg;
  Message recvMsg;
  esp_now_peer_info_t peerInfoDx;
  esp_now_peer_info_t peerInfoSx;

  uint8_t macDx[6];
  uint8_t macSx[6];

  char lastCmdDx[MESSAGE_SIZE];
  char lastCmdSx[MESSAGE_SIZE];

  static DongleController* instance;

  static void onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len);

  void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
  void prepareAndSend(const char* cmd, const uint8_t* mac);
  void handleSerialInput();  // now private
};