#include <esp_now.h>
#include <WiFi.h>
//getting mac address
#include "ESP32-mac.h"

#ifndef ESP32_RECIEVE_PLUS_SEND_H
#define ESP32_RECIEVE_PLUS_SEND_H
#ifdef __cplusplus
extern "C" {
#endif


//external variables
// extern reciever_message_t recievedmessage;
// extern uint8_t atennas_plus_main[6];
//extern sender_message_t sentmessage;
class recieve_plus_send{
    public:
    //0:main 1:atenna_one 2:atenna_two 3:atenna_three 4:atenna_four 5:atenna_five
    recieve_plus_send(uint8_t client_atenna,  uint8_t *targetAddr);
    bool prereqSend();
    bool Recieve();
    bool prereqSend_and_Receive();
    void sendMessage(uint8_t score);
    void printMAC();
    void setTarget(uint8_t *targetAddr);
    uint8_t getMessage();
    uint8_t getLocation();


    //getter
    uint8_t getatennaStatus(uint8_t attenachoice);
    private:
    typedef struct {
  // we can add more if we need to
 // uint8_t task_completion = 0;
  uint8_t atenna_number =0;
  uint8_t score =0;
} sender_message_t, reciever_message_t;
  reciever_message_t recievedmessage;
  sender_message_t sentmessage;
    static recieve_plus_send *s_instance;
    static void onDataSent(const esp_now_send_info_t *receiver_info, esp_now_send_status_t status);
    // esp_now_send_cb_t
    // static void
    static void onDataRecv(const esp_now_recv_info_t *sender_info, const uint8_t *incomingData, int len);
    uint8_t message=0;
    uint8_t location = 0;
    esp_now_peer_info_t targetInfo;
    uint8_t targetAddress[6] ={0};


    bool addReciever();
};


#ifdef __cplusplus
}
#endif
#endif