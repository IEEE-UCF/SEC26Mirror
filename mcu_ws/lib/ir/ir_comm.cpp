#include "ir_comm.h"
// the equivalent of a serial monitor for arduino only this is for esp
#include <cstring>

#include "esp_log.h"

static const char *TAG = "esp32-irNEC";
// constants
#define RMT_RESOLUTION_HZ 1000000  // 1MHz resolution, 1 tick = 1us
#define NEC_DECODE_MARGIN 200
#define NEC_LEADING_CODE_DUR_0 9000
#define NEC_LEADING_CODE_DUR_1 4500
#define NEC_PAYLOAD_ZERO_DUR_0 560
#define NEC_PAYLOAD_ZERO_DUR_1 560
#define NEC_PAYLOAD_ONE_DUR_0 560
#define NEC_PAYLOAD_ONE_DUR_1 1690
namespace RawDrivers {
IRNecSender *IRNecSender::s_instance = nullptr;
IRNecReceiver *IRNecReceiver::s_instance = nullptr;
IRNecSender::~IRNecSender() {
  rmt_disable(s_instance->tx_channel);
  rmt_del_channel(s_instance->tx_channel);
}
bool IRNecSender::init() {
  rmt_tx_channel_config_t tx_cfg = {
      .gpio_num = _setup.receive_send_pin,
      .clk_src = RMT_CLK_SRC_DEFAULT,
      .resolution_hz = RMT_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 4,
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_instance->tx_channel));
  rmt_carrier_config_t carrier_cfg = {
      .frequency_hz = 38000,
      .duty_cycle = .33,
  };
  ESP_ERROR_CHECK(rmt_apply_carrier(s_instance->tx_channel, &carrier_cfg));
  ir_nec_encoder_config_t nec_cfg = {.resolution = RMT_RESOLUTION_HZ};
  ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_cfg, &s_instance->nec_encoder));
  ESP_ERROR_CHECK(rmt_enable(s_instance->tx_channel));
  return true;
}
void IRNecSender::sendMessage(uint16_t address, uint16_t command) {
  uint8_t lower_address = address & 0x0FF;
  uint16_t NEC_address = (lower_address) | ((~address) << 8);
  uint8_t lower_command = command & 0x0FF;
  uint16_t NEC_command = (lower_command) | ((~command) << 8);
  ir_nec_scan_code_t data = {.address = NEC_address, .command = NEC_command};
  rmt_transmit_config_t tx_config = {.loop_count = 0};
  ESP_ERROR_CHECK(rmt_transmit(s_instance->tx_channel, s_instance->nec_encoder,
                               &data, sizeof(data), &tx_config));
  s_instance->IRNecSender_State = SENDING;
}

bool IRNecReceiver::init() {
  s_instance->rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  rmt_rx_channel_config_t rx_cfg = {.gpio_num = _setup.receive_send_pin,
                                    .clk_src = RMT_CLK_SRC_DEFAULT,
                                    .resolution_hz = RMT_RESOLUTION_HZ,
                                    .mem_block_symbols = 64};
  ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &s_instance->rx_channel));
  rmt_rx_event_callbacks_t cbs = {
      .on_recv_done = s_instance->rx_done_callback,
  };
  // IRNecReceiver::rx_done_callback
  ESP_ERROR_CHECK(
      rmt_rx_register_event_callbacks(s_instance->rx_channel, &cbs, this));
  rmt_receive_config_t rx_config = {
      .signal_range_min_ns = 1250,
      .signal_range_max_ns = 12000000,
  };
  ESP_ERROR_CHECK(rmt_enable(s_instance->rx_channel));
  ESP_ERROR_CHECK(rmt_receive(s_instance->rx_channel, s_instance->raw_symbols,
                              sizeof(s_instance->raw_symbols), &rx_config));
  s_instance->received_address = 0;
  s_instance->received_command = 0;
  return true;
}
IRNecReceiver::~IRNecReceiver() {
  if (s_instance->rx_channel) {
    rmt_disable(s_instance->rx_channel);
    rmt_del_channel(s_instance->rx_channel);
  }
  if (s_instance->rx_queue) {
    vQueueDelete(s_instance->rx_queue);
  }
}
bool IRNecReceiver::rx_done_callback(rmt_channel_handle_t channel,
                                     const rmt_rx_done_event_data_t *edata,
                                     void *user_data) {
  IRNecReceiver *self = static_cast<IRNecReceiver *>(user_data);
  BaseType_t high_task_wakeup = pdFALSE;
  xQueueSendFromISR(self->rx_queue, edata, &high_task_wakeup);
  return high_task_wakeup == pdTRUE;
}
bool IRNecReceiver::checkInRange(uint32_t signal_duration,
                                 uint32_t spec_duration) {
  return (signal_duration < (spec_duration + NEC_DECODE_MARGIN)) &&
         (signal_duration > (spec_duration - NEC_DECODE_MARGIN));
}
bool IRNecReceiver::parseLogic0(rmt_symbol_word_t *symbol) {
  return checkInRange(symbol->duration0, NEC_PAYLOAD_ZERO_DUR_0) &&
         checkInRange(symbol->duration1, NEC_PAYLOAD_ZERO_DUR_1);
}
bool IRNecReceiver::parseLogic1(rmt_symbol_word_t *symbol) {
  return checkInRange(symbol->duration0, NEC_PAYLOAD_ONE_DUR_0) &&
         checkInRange(symbol->duration1, NEC_PAYLOAD_ONE_DUR_1);
}
bool IRNecReceiver::parseFrame(rmt_symbol_word_t *symbols, size_t count,
                               uint16_t &out_addr, uint16_t &out_cmd) {
  if (count < 34) return false;
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
    return false;  // Checksum failed! This is noise.
  }

  out_addr = address;
  out_cmd = command;
  s_instance->received_address = address;
  s_instance->received_command = command;
  return true;
}
bool IRNecReceiver::checkMessage(uint16_t &addr, uint16_t &cmd) {
  rmt_rx_done_event_data_t rx_data;
  if (xQueueReceive(s_instance->rx_queue, &rx_data, 0) == pdPASS) {
    bool success =
        parseFrame(rx_data.received_symbols, rx_data.num_symbols, addr, cmd);
    rmt_receive_config_t rx_config = {
        .signal_range_min_ns = 1250,
        .signal_range_max_ns = 12000000,
    };
    ESP_ERROR_CHECK(rmt_receive(s_instance->rx_channel, s_instance->raw_symbols,
                                sizeof(s_instance->raw_symbols), &rx_config));
    s_instance->IRNecReceiver_State = RECEIVING;
    return success;
  }
  s_instance->IRNecReceiver_State = IDLE;
  return false;
}
uint16_t IRNecReceiver::getaddressCommand(uint8_t choice) {
  if (choice == 1) {
    return s_instance->received_address;
  } else if (choice == 0) {
    return s_instance->received_command;
  } else {
    Serial.printf("Param 1: returns address\nParam 2: returns command");
  }
  return 0;
}
void IRNecSender::update() {
  unsigned long currentmicrosecond = micros();
  unsigned long microinterval =
      currentmicrosecond - s_instance->previousmicrosecond;
  static int count = 0;
  if (microinterval >= 60) {
    s_instance->previousmicrosecond = currentmicrosecond;
    if (s_instance->IRNecSender_State == SENDING) {
      count++;
      if (count == 3) {
        count = 0;
        s_instance->IRNecSender_State = IDLE;
      }
    } else {
    }
  }
}
void IRNecReceiver::update() {
  unsigned long currentmicrosecond = micros();
  unsigned long microinterval =
      currentmicrosecond - s_instance->previousmicrosecond;
  if (microinterval >= 60) {
    s_instance->previousmicrosecond = currentmicrosecond;
    if (s_instance->IRNecReceiver_State == RECEIVING) {
      Serial.printf("You've got mail!");
      s_instance->IRNecReceiver_State = IDLE;
    } else {
    }
  }
}
}  // namespace RawDrivers
