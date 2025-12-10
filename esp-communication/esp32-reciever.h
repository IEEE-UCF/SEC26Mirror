#include <esp_now.h>
#include <WiFi.h>
#include "ESP32-mac.h"
#ifndef ESP32_RECIEVER_H
#define ESP32_RECIEVER_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // we can add more if we need to
  //uint8_t task_completion = 0;
  uint8_t atenna_number =0;
} reciever_message_t;

class Reciever{
  public:
  Reciever();
  void checkAtenna();
  void setandprintMAC();
  static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
  void settargetMAC(uint8_t espaddress[6], uint8_t target[]);
  bool preReq();
  private:
  uint8_t macAddress[6];
  uint8_t atennas[5];
};
extern reciever_message_t message;

void preRecieve();
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
#ifdef __cplusplus
}
#endif
#endif