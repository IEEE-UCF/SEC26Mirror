#include "esp32-recieve-plus-send.h"
//external variables for message received and the status for the atennas
reciever_message_t recievedmessage;
uint8_t atennas_plus_main[6]={0};

//constructor
recieve_plus_send::recieve_plus_send(uint8_t client_atenna, uint8_t *targetAddr){
    atenna_or_main = client_atenna;
    memcpy(this->targetAddress, targetAddr, 6);
}
//if you need to change the reciever address
void recieve_plus_send::setTarget(uint8_t *targetAddr){
    memcpy(targetAddr, targetAddress,6);
}
//if necessary here is a function to print out the MAC address of the current esp
void recieve_plus_send::printMAC(){
      esp_err_t ret = esp32_mac_ip_init();
  if(ret!=ESP_OK)
  {
    Serial.printf("Failed to initialize library: %s\n", esp_err_to_name(ret));

  }
  mac_info_t mac_info;
  esp32_mac_ip_get_mac(ESP_MAC_WIFI_STA, &mac_info);
  esp32_ip_print_mac(&mac_info);  
}
//atenna 1-5 are marked in the array if the atenna is one then it completed its task
//else it is yet to be completed
uint8_t recieve_plus_send::getatennaStatus(uint8_t atennachoice){
    return atennas_plus_main[atennachoice];
}
//true or false statement AND makes sures no error code pop
//but it sits and listens....
bool recieve_plus_send::Recieve(){
    WiFi.mode(WIFI_STA);
    if(esp_now_init()!=ESP_OK){
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    esp_now_register_recv_cb(recieve_plus_send::onDataRecv);
    return true;
}
//function to send a message of type struct with associated atenna number
void recieve_plus_send::sendMessage(){
    sentmessage.atenna_number = atenna_or_main;
  esp_err_t result =
      esp_now_send(targetAddress, (uint8_t *)&sentmessage, sizeof(sentmessage));
  if (result == ESP_OK) {
    Serial.printf("\nSent with success");
  } else {
    Serial.println("\nError sending the data");
  }

}
//Initialization of NVS FLASH--if it goes wrong gives back error code
//if not it goes on open the channel and then adds on the target on the list
bool recieve_plus_send::prereqSend(){
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }
  esp_now_register_send_cb(recieve_plus_send::onDataSent);
  memcpy(targetInfo.peer_addr, targetAddress,6);
  targetInfo.channel = 0;
  targetInfo.encrypt = false;
  return addReciever();
}
//private function to add recievers on a list--space is limited
bool recieve_plus_send::addReciever() {
  if (esp_now_add_peer(&targetInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  return true;
}
//function to check if the esp IS sending a message(pov:the sender)
void recieve_plus_send::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success"
                                                : "Delivery Fail");
}
//uses protocols from ESP32 library and then copies the strucutre message into the reciever structure
//we then set the atenna number which comes from the message into our array of atennas (pov: the reciever)
void recieve_plus_send::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
    if(len!=sizeof(recievedmessage)){
        Serial.println("Err: Size mismatch");
        return;
    }
    memcpy(&recievedmessage, incomingData, sizeof(recievedmessage));
    if(recievedmessage.atenna_number>0)
    {
        Serial.printf("\nAtenna number %d is done!", recievedmessage.atenna_number);
        atennas_plus_main[recievedmessage.atenna_number] = 1;
    }else{
        Serial.printf("\nnothing to see!");
    }
}