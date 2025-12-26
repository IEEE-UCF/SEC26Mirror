/*
  @ir-communication
  @author Jonathan Hernandez-Morales
  @Date 12/20/2025
*/
#pragma once
#include <Arduino.h>

#include <cstdint>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#ifndef ESP32_IRNEC_HPP
#define ESP32_IRNEC_HPP
extern "C" {
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "ir_nec_encoder.h"
}
#define antenna_one 0x00
#define antenna_two 0x30
#define antenna_three 0x50
#define antenna_four 0x60

#define red 0x09
#define green 0x0A
#define blue 0x0C
#define purple 0x0F
#include "BaseDriver.h"
namespace RawDrivers {
//@brief configuration setup for IRsender/receiver
class IRNecSetup : public Classes::BaseSetup {
 public:
  IRNecSetup(const char *_id, const gpio_num_t pin) : Classes::BaseSetup(_id) {
    receive_send_pin = pin;
  }
  gpio_num_t receive_send_pin;
};
//@brief base implementation for ir-sender
class IRNecSender : public Classes::BaseDriver {
 public:
  ~IRNecSender() override = default;
  IRNecSender(const IRNecSetup &setup)
      : Classes::BaseDriver(setup), _setup(setup) {}
  //@brief capabilities to send an ir-signal
  void sendMessage(uint16_t address, uint16_t command);
  //@brief must intialize transmission channel before sending signal
  bool init() override;
  //@brief changes states, will send 3 signals then will go IDLE
  void update() override;

 private:
  enum state { SENDING, IDLE } IRNecSender_State;
  unsigned long previousmicrosecond = 0;
  const IRNecSetup &_setup;
  rmt_channel_handle_t tx_channel = NULL;
  rmt_encoder_handle_t nec_encoder = NULL;
  void initChannel(gpio_num_t pin);
  static IRNecSender *s_instance;
};
//@brief base implementation for ir-receiver
class IRNecReceiver : public Classes::BaseDriver {
 public:
  ~IRNecReceiver() override = default;
  IRNecReceiver(const IRNecSetup &setup)
      : Classes::BaseDriver(setup), _setup(setup) {}
  //@breif intialize the receving channel and allow a background task queue for
  // receiving
  bool init() override;
  //@brief returns either true or false if received a message or not
  bool checkMessage(uint16_t &addr, uint16_t &cmd);
  //@brief an indicator for YOU to know if something was received
  void update() override;
  //@brief depending on your choice you get to see what was sent
  // address=antenna /////////command=color
  uint16_t getaddressCommand(uint8_t choice);

 private:
  enum state { RECEIVING, IDLE } IRNecReceiver_State;
  unsigned long previousmicrosecond = 0;
  const IRNecSetup &_setup;
  static IRNecReceiver *s_instance;
  uint16_t received_address;
  uint16_t received_command;
  rmt_channel_handle_t rx_channel = NULL;
  QueueHandle_t rx_queue = NULL;
  rmt_symbol_word_t raw_symbols[64];
  static bool rx_done_callback(rmt_channel_handle_t channel,
                               const rmt_rx_done_event_data_t *edata,
                               void *user_data);

  bool parseFrame(rmt_symbol_word_t *symbols, size_t count, uint16_t &addr,
                  uint16_t &cmd);
  bool checkInRange(uint32_t signal_duration, uint32_t spec_duration);
  bool parseLogic0(rmt_symbol_word_t *symbol);
  bool parseLogic1(rmt_symbol_word_t *symbol);
};
}  // namespace RawDrivers
#endif