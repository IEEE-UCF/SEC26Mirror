#include "ESP32-mac.h"
#include <WiFi.h>
#include <esp_now.h>

#ifndef ESP32_SENDER_H
#define ESP32_SENDER_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  // we can add more if we need to
 // uint8_t task_completion = 0;
  uint8_t atenna_number =0;
} sender_message_t;

class sender{
public:
  // constructor with reciever address and associated "tag" for the sender
  sender(int atenna, uint8_t *receiverAddr);
  // functions to conduct
  static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  //void addReciever(esp_now_peer_info_t *recieverPeer, uint8_t address[]);
  bool preReq();
  void sendMessage();
  uint8_t getatennaNum();
  private:
  int atenna_num=0;
  uint8_t recieverAddress[6]={0};
  esp_now_peer_info_t recieverInfo;
  sender_message_t message;

  bool addReciever();
};

// external variables


#ifdef __cplusplus
}
#endif
#endif