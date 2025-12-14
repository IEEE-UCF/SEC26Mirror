#pragma once
#include <cstdint>
//pin memory addresses defined already in the ESP32
#include "driver/gpio.h"
//esp32 runs on FreeRTOS we can "split" task with these headers
#include "freertos/FreeRTOS.h"
//whatever we get from our reciever side we'll immediately dump it inside a queue
#include "freertos/queue.h"
#ifndef ESP32_IRNEC_HPP
#define ESP32_IRNEC_HPP
extern "C"{
    //transmission capabilites via RMT peripheral
    #include "driver/rmt_tx.h"
    //receiving capabilites via RMT peripheral
    #include "driver/rmt_rx.h"
    //encoder file from official espidf repo online
    #include "ir_nec_encoder.h"
}
//address and command
#define antenna_one 0x00
#define antenna_two 0x30
#define antenna_three 0x50
#define antenna_four 0x60

#define red 0x09
#define green 0x0A
#define blue 0x0C
#define purple 0x0F
class IRSender{
    public:
    //based on the gpio.h pins 
    IRSender(gpio_num_t pin);
    ~IRSender();//this is a destructor, can;t use up all the memory!!
    
    //for antenna-->address and then for color--> command
    void send(uint16_t address, uint16_t command);
    private:
    //transmission channel based rmt_types
    rmt_channel_handle_t tx_channel =NULL;
    //an object that handles our encoder
    rmt_encoder_handle_t nec_encoder = NULL;

    //intialize your pin!
    void initChannel(gpio_num_t pin);
};
class IRReceiver{
    public:
    IRReceiver(gpio_num_t pin);
    ~IRReceiver();

    //pointers to the actual address, and commands for the receiver, returns 
    //either true or false if they match....
    bool checkMessage(uint16_t &addr, uint16_t &cmd);
    private:
    
    rmt_channel_handle_t rx_channel = NULL;
    QueueHandle_t rx_queue =NULL;
    rmt_symbol_word_t raw_symbols[64];//Buffer for raw pulses???

    static bool rx_done_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data);
    
    
    //Functions based on espidf transreciever code 
    bool parseFrame(rmt_symbol_word_t *symbols, size_t count, uint16_t &addr, uint16_t &cmd);
    bool checkInRange(uint32_t signal_duration, uint32_t spec_duration);
    bool parseLogic0(rmt_symbol_word_t *symbol);
    bool parseLogic1(rmt_symbol_word_t *symbol);
};
#endif