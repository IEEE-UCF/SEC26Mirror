#include "esp32-reciever.h"
//message structure and reciever structure must be the same.....
reciever_message_t recieverend;
//if by choice find and set the mac address to a public array that contains the hexadecimals for the MAC address
void Reciever::setandprintMAC(){
  esp_err_t ret = esp32_mac_ip_init();
  if(ret!=ESP_OK)
  {
    Serial.printf("Failed to initialize library: %s\n", esp_err_to_name(ret));

  }
  mac_info_t mac_info;
  esp32_mac_ip_get_mac(ESP_MAC_WIFI_STA, &mac_info);
  esp32_ip_print_mac(&mac_info);
  memcpy(mac_info.mac,macAddress,6);
}
//if desired to get the actual mac address user will input their own array to have a "copy" of the MAC address of the reciever
void Reciever::settargetMAC(uint8_t espaddress[6], uint8_t target[]){
    memcpy(espaddress,target,6);
}
//true or false statement AND makes sures no error code pop
bool Reciever::preReq(){
    WiFi.mode(WIFI_STA);
    if(esp_now_init()!=ESP_OK){
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    esp_now_register_recv_cb(Reciever::onDataRecv);
    return true;
}
//using our reciever structure which contains the atenna number we can make a list of attenas
//that 1: completed task  0: not completed
void Reciever::checkAtenna(){
    atennas[recieverend.atenna_number-1]=1;
}
//uses protocols from ESP32 library and then copies the strucutre message into the reciever structure
void Reciever::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len){
    if(len!=sizeof(recieverend)){
        Serial.println("Err: Size mismatch");
        return;
    }
    memcpy(&recieverend, incomingData, sizeof(recieverend));
    if(recieverend.atenna_number>0)
    {
        Serial.println("HOORAY!");
    }else{
        Serial.println("....:(....)");
    }
}