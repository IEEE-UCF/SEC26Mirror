/*
@file-ESP_NOW_COMMUNICATE_NOW!
@date 12/19/2025
@author Jonathan Hernandez-Morales
*/
#include <WiFi.h>
#include <esp_now.h>
// getting mac address
#include "BaseDriver.h"
#include "ESP32-mac.h"
#ifndef ESP32_RECIEVE_PLUS_SEND_H
#define ESP32_RECIEVE_PLUS_SEND_H
#ifdef __cplusplus
extern "C" {
#endif

namespace RawDrivers {
//@brief setup for esp-now communication
class ESPNowSetup : public Classes::BaseSetup {
 public:
  ESPNowSetup(const char *_id, const uint8_t *targetMac)
      : Classes::BaseSetup(_id) {
    memcpy(targetAddress, targetMac, 6);
  }
  uint8_t targetAddress[6];
};
//@breif main constructor for esp-now communication
class EspNow : public Classes::BaseDriver {
 public:
  ~EspNow() override = default;
  EspNow(const ESPNowSetup &setup)
      : Classes::BaseDriver(setup), _setup(setup) {}
  //@breif intialize and set up esp for receving and sending signals with one
  // another including adding peers
  bool init() override;
  //@brief one of the message structs contain the score of which earth will send
  // this is how it'll do it
  void sendScore(uint8_t score);
  //@breif in case you need to know the MAC address of the current-esp
  void printMAC();
  //@brief can update later if need be
  void update() override;
  uint8_t getScore();

 private:
  unsigned long previousmicrosecond = 0;
  enum state { IDLE, SENDING, RECEIVED } esp_state;
  const ESPNowSetup &_setup;
  //@brief callback functions!
  // static void onDataSent(const esp_now_send_info_t *receiver_info,
  // esp_now_send_status_t status); static void onDataRecv(const
  // esp_now_recv_info_t *sender_info, const uint8_t *incomingData, int len);
  // Change these lines:
  static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
  static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData,
                         int len);
  static EspNow *s_instance;
  //@breif both sender and receiver will recieve/send this information...
  typedef struct {
    uint8_t atenna_number = 0;
    uint8_t score = 0;
  } sender_message_t, reciever_message_t;
  uint8_t received_score = 0;
  reciever_message_t recievedmessage;
  sender_message_t sentmessage;
};
}  // namespace RawDrivers
#ifdef __cplusplus
}
#endif
#endif
