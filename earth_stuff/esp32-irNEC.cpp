#include "esp32-irNEC.hpp"
// the equivalent of a serial monitor for arduino only this is for esp
#include "esp_log.h"
#include <cstring>

static const char *TAG = "esp32-irNEC";
// constants
#define RMT_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define NEC_DECODE_MARGIN 200
#define NEC_LEADING_CODE_DUR_0 9000
#define NEC_LEADING_CODE_DUR_1 4500
#define NEC_PAYLOAD_ZERO_DUR_0 560
#define NEC_PAYLOAD_ZERO_DUR_1 560
#define NEC_PAYLOAD_ONE_DUR_0 560
#define NEC_PAYLOAD_ONE_DUR_1 1690
// address and commands

//************************************//
//***IR_SENDER CAPABILITES***//
//************************************//
IRSender::IRSender(gpio_num_t pin) { initChannel(pin); }
IRSender::~IRSender() {
  if (tx_channel) {
    rmt_disable(tx_channel);
    rmt_del_channel(tx_channel);
  }
}
// we fill up some values in the structures and do a
// safety check on each configuration
// if any errors pop up we IMMEDIATELY halt the processor
void IRSender::initChannel(gpio_num_t pin) {
  // a structure for our RMT peripheral!
  // defined a clock source, its frequency, the PIN we are going to use
  // we want to specify what resources we want to use...
  // so we set up some values for them
  rmt_tx_channel_config_t tx_cfg = {
      .gpio_num = pin,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 4,
  };
  // safety checks
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_channel));

  // after specifying our hardware resources we want to be able to
  // turn "on and off" our signal to a standard 38kHz-->NEC PROTOCOL
  rmt_carrier_config_t carrier_cfg = {
      .frequency_hz = 38000,
      .duty_cycle = .33,
  };

  ESP_ERROR_CHECK(rmt_apply_carrier(tx_channel, &carrier_cfg));

  // then we want to set up the encoder it basically handles the pulses
  // in which according to our rmt configuration above is
  // RMT_RESOLUTION_HZ
  ir_nec_encoder_config_t nec_cfg = {.resolution = RMT_RESOLUTION_HZ};
  ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_cfg, &nec_encoder));

  ESP_ERROR_CHECK(rmt_enable(tx_channel));
}

void IRSender::send(uint16_t address, uint16_t command) {
  // structure straight from the encoder file...
  uint8_t lower_address = address & 0x0FF;
  // send a 16 bit "address" lower 8 is address and upper 8 is
  // ~address(opposite) via NEC protocol
  uint16_t NEC_address = (lower_address) | ((~address) << 8);
  uint8_t lower_command = command & 0x0FF;
  // send a 16 bit "command" lower 8 is address and upper 8 is
  // ~command(opposite) via NEC protocol
  uint16_t NEC_command = (lower_command) | ((~command) << 8);
  ir_nec_scan_code_t data = {.address = NEC_address, .command = NEC_command};
  rmt_transmit_config_t tx_config = {.loop_count = 0};
  // takes data from the data strucuture you just created
  // sends it to the encoder, our encoder uses the 1MHz to calculate
  // tne measurements of pulses depending on the hexadecimal that was sent

  // sends the entire transmission AS ONE BYTE
  // so header/address/~address/command/~command
  // should be in here to account for that
  ESP_ERROR_CHECK(
      rmt_transmit(tx_channel, nec_encoder, &data, sizeof(data), &tx_config));
}

//************************************//
//***IR_Receiver CAPABILITES***//
//************************************//
IRReceiver::IRReceiver(gpio_num_t pin) {
  // create queue
  rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  // create RX channel
  rmt_rx_channel_config_t rx_cfg = {.gpio_num = pin,
                                    .clk_src = RMT_CLK_SRC_DEFAULT,
                                    .resolution_hz = RMT_RESOLUTION_HZ,
                                    .mem_block_symbols = 64};
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &rx_channel));

  // register callback
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = IRReceiver::rx_done_callback,
  };

  ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, this));
  // config and enable
  rmt_receive_config_t rx_config = {
      .signal_range_min_ns = 1250,
      .signal_range_max_ns = 12000000,
  };
  ESP_ERROR_CHECK(rmt_enable(rx_channel));

  // ready to receive
  ESP_ERROR_CHECK(
      rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));
  received_address = 0;
  received_command = 0;
}
IRReceiver::~IRReceiver() {
  if (rx_channel) {
    rmt_disable(rx_channel);
    rmt_del_channel(rx_channel);
  }
  if (rx_queue) {
    vQueueDelete(rx_queue);
  }
}
uint16_t IRReceiver::getaddressCommand(uint8_t choice) {
  if (choice == 1) {
    return received_address;
  } else if (choice == 0) {
    return received_command;
  } else {
    return -1;
  }
}
bool IRReceiver::rx_done_callback(rmt_channel_handle_t channel,
                                  const rmt_rx_done_event_data_t *edata,
                                  void *user_data) {
  IRReceiver *self = static_cast<IRReceiver *>(user_data);
  BaseType_t high_task_wakeup = pdFALSE;
  xQueueSendFromISR(self->rx_queue, edata, &high_task_wakeup);
  return high_task_wakeup == pdTRUE;
}
bool IRReceiver::checkMessage(uint16_t &addr, uint16_t &cmd) {
  rmt_rx_done_event_data_t rx_data;
  if (xQueueReceive(rx_queue, &rx_data, 0) == pdPASS) {
    // Attempt to parse
    bool success =
        parseFrame(rx_data.received_symbols, rx_data.num_symbols, addr, cmd);

    // Re-arm the receiver for the next message
    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };
    ESP_ERROR_CHECK(
        rmt_receive(rx_channel, raw_symbols, sizeof(raw_symbols), &rx_config));

    return success;
  }
  return false;
}

bool IRReceiver::checkInRange(uint32_t duration, uint32_t spec) {
  return (duration < (spec + NEC_DECODE_MARGIN)) &&
         (duration > (spec - NEC_DECODE_MARGIN));
}
bool IRReceiver::parseLogic0(rmt_symbol_word_t *s) {
  return checkInRange(s->duration0, NEC_PAYLOAD_ZERO_DUR_0) &&
         checkInRange(s->duration1, NEC_PAYLOAD_ZERO_DUR_1);
}
bool IRReceiver::parseLogic1(rmt_symbol_word_t *s) {
  return checkInRange(s->duration0, NEC_PAYLOAD_ONE_DUR_0) &&
         checkInRange(s->duration1, NEC_PAYLOAD_ONE_DUR_1);
}
bool IRReceiver::parseFrame(rmt_symbol_word_t *symbols, size_t count,
                            uint16_t &out_addr, uint16_t &out_cmd) {
  if (count < 34)
    return false;
  rmt_symbol_word_t *cur = symbols;
  // Check Header
  if (!checkInRange(cur->duration0, NEC_LEADING_CODE_DUR_0) ||
      !checkInRange(cur->duration1, NEC_LEADING_CODE_DUR_1)) {
    return false;
  }
  cur++;

  uint16_t address = 0;
  uint16_t command = 0;

  // Parse Address (16 bits)
  for (int i = 0; i < 16; i++) {
    if (parseLogic1(cur))
      address |= (1 << i);
    else if (parseLogic0(cur))
      address &= ~(1 << i);
    else
      return false;
    cur++;
  }

  // Parse Command (16 bits)
  for (int i = 0; i < 16; i++) {
    if (parseLogic1(cur))
      command |= (1 << i);
    else if (parseLogic0(cur))
      command &= ~(1 << i);
    else
      return false;
    cur++;
  }
  uint8_t cmd_data = command & 0xFF;
  uint8_t cmd_inv = (command >> 8) & 0xFF;

  if (cmd_data != (uint8_t)(~cmd_inv)) {
    return false; // Checksum failed! This is noise.
  }

  out_addr = address;
  out_cmd = command;
  received_address = address;
  received_command = command;

  return true;
}