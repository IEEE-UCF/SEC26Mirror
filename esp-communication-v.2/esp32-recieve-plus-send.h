#include <esp_now.h>
#include <WiFi.h>
//getting mac address
#include "ESP32-mac.h"


#ifndef ESP32_RECIEVE_PLUS_SEND_H
#define ESP32_RECIEVE_PLUS_SEND_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  // we can add more if we need to
 // uint8_t task_completion = 0;
  uint8_t atenna_number =0;
} sender_message_t, reciever_message_t;

//external variables
extern reciever_message_t recievedmessage;
extern uint8_t atennas_plus_main[6];
//extern sender_message_t sentmessage;
class recieve_plus_send{
    public:
    //0:main 1:atenna_one 2:atenna_two 3:atenna_three 4:atenna_four 5:atenna_five
    recieve_plus_send(uint8_t client_atenna,  uint8_t *targetAddr);
    bool prereqSend();
    bool Recieve();
    void sendMessage();
    void printMAC();
    void setTarget(uint8_t *targetAddr);

    //getter
    uint8_t getatennaStatus(uint8_t attenachoice);
    private:
    static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
    static void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
    uint8_t atenna_or_main=0;
    esp_now_peer_info_t targetInfo;
    uint8_t targetAddress[6] ={0};
    sender_message_t sentmessage;

    bool addReciever();
};


#ifdef __cplusplus
}
#endif
#endif