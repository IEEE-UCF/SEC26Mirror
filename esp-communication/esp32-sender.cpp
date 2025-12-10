#include "esp32-sender.h"
//construcutor will implement a  basic one way sender
sender::sender(int atenna, uint8_t *receiverAddr){
  atenna_num = atenna;
// MAC address of Reciever: 44:1D:64:F3:B2:04 <--this is an example
  memcpy(this->recieverAddress,receiverAddr,6);
}

//get the senders atenna number
uint8_t sender::getatennaNum(){
  return atenna_num;
}

//Initialization of NVS FLASH--if it goes wrong gives back error code
bool sender::preReq() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(sender::onDataSent);
  memcpy(recieverInfo.peer_addr, recieverAddress,6);
  recieverInfo.channel = 0;
  recieverInfo.encrypt = false;
  return addReciever();
}

//function to check if the esp IS sending a message(pov:the sender)
void sender::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
}
//private function to add recievers on a list--space is limited
bool sender::addReciever() {
  if (esp_now_add_peer(&recieverInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  return true;
}

//function to send a struct of task completed and the associated atenna number
void sender::sendMessage() {
//  message.task_completion = 1;
  message.atenna_number = atenna_num;
  esp_err_t result =
      esp_now_send(recieverAddress, (uint8_t *)&message, sizeof(message));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

