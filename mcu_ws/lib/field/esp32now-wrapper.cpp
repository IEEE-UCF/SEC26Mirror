#include "esp32now-wrapper.h"
namespace RawDrivers {
EspNow *EspNow::s_instance = nullptr;
bool EspNow::init() {
  s_instance = this;
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
    return false;
  esp_now_register_recv_cb(EspNow::onDataRecv);
  esp_now_register_send_cb(EspNow::onDataSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, _setup.targetAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  return (esp_now_add_peer(&peerInfo) == ESP_OK);
}
// void EspNow::onDataRecv(const esp_now_recv_info_t *sender_info,
//                         const uint8_t *incomingData, int len) {
//   if (s_instance != nullptr) {
//     if (len != sizeof(s_instance->recievedmessage)) {
//       Serial.println("Err: Size mismatch");
//       s_instance->esp_state = IDLE;
//       return;
//     }
//     memcpy(&s_instance->recievedmessage, incomingData,
//            sizeof(s_instance->recievedmessage));
//     s_instance->esp_state = RECEIVED;
//   }
// }
void EspNow::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (s_instance != nullptr) {
    if (len != sizeof(s_instance->recievedmessage)) {
      Serial.println("Err: Size mismatch");
      s_instance->esp_state = IDLE;
      return;
    }
    memcpy(&s_instance->recievedmessage, incomingData,
           sizeof(s_instance->recievedmessage));
    s_instance->esp_state = RECEIVED;
  }
}
<<<<<<< HEAD
// void EspNow::onDataSent(const esp_now_send_info_t *receiver_info,
//                         esp_now_send_status_t status) {
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
//                                                 : "Delivery Fail");
//   s_instance->esp_state = SENDING;
// }
void EspNow::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
=======
void EspNow::onDataSent(const uint8_t *mac_addr,
                        esp_now_send_status_t status) {
>>>>>>> 5730874294bfdee94623e9c745fe01cc3b4e3758
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
  s_instance->esp_state = SENDING;
}

void EspNow::sendScore(uint8_t score) {
  sentmessage.score = score;
  esp_err_t result = esp_now_send(_setup.targetAddress, (uint8_t *)&sentmessage,
                                  sizeof(sentmessage));
  if (result == ESP_OK) {
    Serial.printf("\nSent with success");
  } else {
    Serial.println("\nError sending the data");
  }
}
void EspNow::printMAC() {
  esp_err_t ret = esp32_mac_ip_init();
  if (ret != ESP_OK) {
    Serial.printf("Failed to initialize library: %s\n", esp_err_to_name(ret));
  }
  mac_info_t mac_info;
  esp32_mac_ip_get_mac(ESP_MAC_WIFI_STA, &mac_info);
  esp32_ip_print_mac(&mac_info);
}
void EspNow::update() {
  static int count = 0;
  unsigned long currentmicrosecond = micros();
  unsigned long microinterval = currentmicrosecond - previousmicrosecond;
  if (microinterval >= 60) {
    previousmicrosecond = currentmicrosecond;
    if (s_instance->esp_state == RECEIVED) {
      s_instance->received_score = s_instance->recievedmessage.score;
    } else if (s_instance->esp_state == SENDING) {
      count++;
      if (count == 3) {
        s_instance->esp_state = IDLE;
        count = 0;
      }
    } else if (s_instance->esp_state == IDLE) {
    }
  }
}
uint8_t EspNow::getScore() { return s_instance->received_score; }
} // namespace RawDrivers