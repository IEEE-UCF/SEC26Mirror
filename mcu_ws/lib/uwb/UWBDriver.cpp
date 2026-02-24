/**
 * @file UWBDriver.cpp
 * @brief Implementation of DW3000 UWB driver
 * @author SEC26 Team
 * @date 12/22/2025
 */

#include "UWBDriver.h"

#include <SPI.h>

// Global IRQ flag — set by hardware ISR, cleared by the driver after consuming
// the event.
volatile bool g_uwb_irq_fired = false;

void IRAM_ATTR uwb_irq_handler() { g_uwb_irq_fired = true; }

namespace Drivers {

bool UWBDriver::init() {
  // Initialize DW3000
  DW3000.spiSelect(setup_.cs_pin);
  DW3000.begin();

  // Hardware reset if pin is specified
  if (setup_.rst_pin != 255) {
    pinMode(setup_.rst_pin, OUTPUT);
    digitalWrite(setup_.rst_pin, LOW);
    delay(10);
    digitalWrite(setup_.rst_pin, HIGH);
    delay(200);
  } else {
    DW3000.hardReset();
    delay(200);
  }

  // Check SPI connection
  if (!DW3000.checkSPI()) {
    data_.initialized = false;
    return false;
  }

  // Wait for IDLE
  uint8_t idle_attempts = 0;
  while (!DW3000.checkForIDLE() && idle_attempts < 10) {
    delay(100);
    idle_attempts++;
  }
  if (idle_attempts >= 10) {
    data_.initialized = false;
    return false;
  }

  // Soft reset
  DW3000.softReset();
  delay(200);

  // Check IDLE again
  if (!DW3000.checkForIDLE()) {
    data_.initialized = false;
    return false;
  }

  // Initialize chip
  DW3000.init();
  DW3000.setupGPIO();
  DW3000.configureAsTX();

  // Set device ID
  DW3000.setSenderID(setup_.device_id);

  // Clear status
  DW3000.clearSystemStatus();

  data_.mode = setup_.mode;
  data_.device_id = setup_.device_id;
  data_.initialized = true;

  // Attach hardware interrupt if IRQ pin is wired
  if (setup_.irq_pin != 255) {
    pinMode(setup_.irq_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(setup_.irq_pin), uwb_irq_handler,
                    RISING);
  }

  return true;
}

void UWBDriver::begin() {
  if (!data_.initialized) {
    return;
  }

  // Initialize mode-specific state
  if (data_.mode == UWBMode::ANCHOR) {
    // Anchor starts in RX mode waiting for ranging requests
    anchor_stage_ = 0;
    DW3000.standardRX();
  } else {
    // Tag starts idle, will initiate ranging when requested
    tag_stage_ = 0;
    current_anchor_index_ = 0;
  }
}

void UWBDriver::update() {
  if (!data_.initialized) {
    return;
  }

  // Update temperature periodically
  static elapsedMillis temp_timer;
  if (temp_timer > 1000) {
    data_.temperature = DW3000.getTempInC();
    temp_timer = 0;
  }

  // Update based on mode
  if (data_.mode == UWBMode::TAG) {
    updateTagMode();
  } else {
    updateAnchorMode();
  }
}

void UWBDriver::pause() {
  // Stop any ongoing operations
  if (data_.mode == UWBMode::ANCHOR) {
    anchor_stage_ = 0;
    inter_beacon_stage_ = 0;
  } else {
    tag_stage_ = 0;
  }
}

void UWBDriver::reset() {
  pause();
  data_.ranging_count = 0;
  data_.error_count = 0;
  data_.num_valid_ranges = 0;
  num_peer_ranges_ = 0;
}

const char* UWBDriver::getInfo() {
  static char info[64];
  const char* mode_str = (data_.mode == UWBMode::TAG) ? "TAG" : "ANCHOR";
  snprintf(info, sizeof(info), "UWB %s ID=%d Init=%d", mode_str,
           data_.device_id, data_.initialized);
  return info;
}

void UWBDriver::setTargetAnchors(const uint8_t* anchor_ids, uint8_t count) {
  num_target_anchors_ = min(count, (uint8_t)UWBDriverData::MAX_ANCHORS);
  for (uint8_t i = 0; i < num_target_anchors_; i++) {
    target_anchors_[i] = anchor_ids[i];
  }
  current_anchor_index_ = 0;
}

bool UWBDriver::startRanging() {
  if (data_.mode != UWBMode::TAG || num_target_anchors_ == 0) {
    return false;
  }

  if (tag_stage_ == 0) {
    // Start ranging to current anchor
    DW3000.setDestinationID(target_anchors_[current_anchor_index_]);
    tag_stage_ = 1;
    tag_timeout_ = 0;
    return true;
  }

  return false;  // Already in progress
}

void UWBDriver::updateTagMode() { processTagStage(); }

// ---- TAG mode — copied from dw3000_doublesided_ranging_ping.ino ----
//
// stage mapping vs example:
//   0 = idle (waiting for startRanging())
//   1 = example case 0 — send frame 1
//   2 = example case 1 — await response (stage 2)
//   3 = example case 2 — send frame 3
//   4 = example case 3 — await final response
//   5 = example case 4 — calculate distance
void UWBDriver::processTagStage() {
  int rx_status;

  // Timeout guard
  if (tag_stage_ > 0 && tag_timeout_ > TAG_TIMEOUT_MS) {
    handleError("TAG timeout");
    tag_stage_ = 0;
    current_anchor_index_ = (current_anchor_index_ + 1) % num_target_anchors_;
    return;
  }

  switch (tag_stage_) {
    case 0:
      // Idle — waiting for startRanging() call
      break;

    case 1:
      // example case 0: start ranging
      t_roundA_ = 0;
      t_replyA_ = 0;
      DW3000.ds_sendFrame(1);
      tag_tx_ = DW3000.readTXTimestamp();
      tag_stage_ = 2;
      break;

    case 2:
      // example case 1: await first response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            tag_stage_ = 0;
          } else if (DW3000.ds_getStage() != 2) {
            DW3000.ds_sendErrorFrame();
            tag_stage_ = 0;
          } else {
            tag_stage_ = 3;
          }
        } else {
          DW3000.clearSystemStatus();
          tag_stage_ = 0;
        }
      }
      break;

    case 3:
      // example case 2: response received, send second ranging
      tag_rx_ = DW3000.readRXTimestamp();
      DW3000.ds_sendFrame(3);
      t_roundA_ = tag_rx_ - tag_tx_;
      tag_tx_ = DW3000.readTXTimestamp();
      t_replyA_ = tag_tx_ - tag_rx_;
      tag_stage_ = 4;
      break;

    case 4:
      // example case 3: await final response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            tag_stage_ = 0;
          } else {
            clock_offset_ = DW3000.getRawClockOffset();
            tag_stage_ = 5;
          }
        } else {
          DW3000.clearSystemStatus();
          tag_stage_ = 0;
        }
      }
      break;

    case 5: {
      // example case 4: calculate distance
      int ranging_time =
          DW3000.ds_processRTInfo(t_roundA_, t_replyA_, DW3000.read(0x12, 0x04),
                                  DW3000.read(0x12, 0x08), clock_offset_);
      float distance = DW3000.convertToCM(ranging_time);

      uint8_t anchor_id = target_anchors_[current_anchor_index_];
      UWBRangeMeasurement& range = data_.ranges[current_anchor_index_];
      range.peer_id = anchor_id;
      range.distance_cm = distance;
      range.clock_offset = clock_offset_;
      range.tx_timestamp = tag_tx_;
      range.rx_timestamp = tag_rx_;
      range.valid = true;
      range.error_code = 0;

      data_.ranging_count++;
      data_.num_valid_ranges++;

      current_anchor_index_ = (current_anchor_index_ + 1) % num_target_anchors_;
      tag_stage_ = 0;
      delay(RANGING_DELAY_MS);
      break;
    }

    default:
      tag_stage_ = 0;
      break;
  }
}

void UWBDriver::setPeerBeacons(const uint8_t* peer_ids, uint8_t count,
                               uint32_t interval_ms) {
  num_peer_beacons_ = min(count, (uint8_t)UWBDriverData::MAX_ANCHORS);
  for (uint8_t i = 0; i < num_peer_beacons_; i++) {
    peer_beacons_[i] = peer_ids[i];
    peer_ranges_[i] = UWBRangeMeasurement{};
  }
  inter_beacon_interval_ms_ = interval_ms;
  current_peer_index_ = 0;
  inter_beacon_timer_ = 0;
}

void UWBDriver::updateAnchorMode() {
  if (num_peer_beacons_ > 0) {
    // If an inter-beacon exchange is in progress, drive it
    if (inter_beacon_stage_ > 0) {
      processInterBeaconStage();
      return;
    }
    // When anchor is idle and the interval has elapsed, start a peer exchange.
    // Must force DW3000 to IDLE first: standardRX() leaves the chip in RX
    // mode, and TX fast commands require IDLE state.
    if (anchor_stage_ == 0 &&
        inter_beacon_timer_ >= inter_beacon_interval_ms_) {
      Serial.printf("[UWB] IB start: peer=%d\n",
                    peer_beacons_[current_peer_index_]);
      Serial.flush();
      dw3000ForceIdle();
      DW3000.setDestinationID(peer_beacons_[current_peer_index_]);
      inter_beacon_stage_ = 1;
      tag_timeout_ = 0;
      inter_beacon_timer_ = 0;
      return;
    }
  }
  processAnchorStage();
}

// ---- ANCHOR mode — copied from dw3000_doublesided_ranging_pong.ino ----
//
// The chip is kept in RX mode between exchanges via TXInstantRX() inside
// ds_sendFrame() and ds_sendRTInfo(). standardRX() is only called explicitly
// in error paths that abort mid-exchange.
void UWBDriver::processAnchorStage() {
  int rx_status;

  switch (anchor_stage_) {
    case 0:
      // example case 0: await ranging request
      t_roundB_ = 0;
      t_replyB_ = 0;
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            anchor_stage_ = 0;
            DW3000.standardRX();
          } else if (DW3000.ds_getStage() != 1) {
            DW3000.ds_sendErrorFrame();
            DW3000.standardRX();
            anchor_stage_ = 0;
          } else {
            anchor_stage_ = 1;
          }
        } else {
          DW3000.clearSystemStatus();
          DW3000.standardRX();
          anchor_stage_ = 0;
        }
      }
      break;

    case 1:
      // example case 1: ranging received, send response
      // Capture sender ID *before* TX so the response is addressed correctly.
      // Without setDestinationID() the DW3000 destination defaults to 0 and
      // the initiator's frame filter rejects frame 2, causing a stage=2
      // timeout.
      data_.last_range.peer_id = DW3000.getSenderID();
      DW3000.setDestinationID(data_.last_range.peer_id);
      DW3000.ds_sendFrame(2);
      anchor_rx_ = DW3000.readRXTimestamp();
      anchor_tx_ = DW3000.readTXTimestamp();
      t_replyB_ = anchor_tx_ - anchor_rx_;
      anchor_stage_ = 2;
      break;

    case 2:
      // example case 2: await second ranging
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            anchor_stage_ = 0;
            DW3000.standardRX();
          } else if (DW3000.ds_getStage() != 3) {
            DW3000.ds_sendErrorFrame();
            DW3000.standardRX();
            anchor_stage_ = 0;
          } else {
            anchor_stage_ = 3;
          }
        } else {
          DW3000.clearSystemStatus();
          DW3000.standardRX();
          anchor_stage_ = 0;
        }
      }
      break;

    case 3:
      // example case 3: send RTInfo — ds_sendRTInfo calls TXInstantRX
      // internally so chip returns to RX automatically; no standardRX() needed
      anchor_rx_ = DW3000.readRXTimestamp();
      t_roundB_ = anchor_rx_ - anchor_tx_;
      DW3000.ds_sendRTInfo(t_roundB_, t_replyB_);
      data_.last_range.valid = true;
      data_.ranging_count++;
      anchor_stage_ = 0;
      break;

    default:
      anchor_stage_ = 0;
      DW3000.standardRX();
      break;
  }
}

// ---- Inter-beacon initiator — copied from dw3000_doublesided_ranging_ping.ino
// ----
//
// Beacon with lower ID plays TAG role toward higher-ID peers.
// stage mapping vs example:
//   1 = example case 0 — send frame 1
//   2 = example case 1 — await response (stage 2)
//   3 = example case 2 — send frame 3
//   4 = example case 3 — await final response
//   5 = example case 4 — calculate distance
void UWBDriver::processInterBeaconStage() {
  int rx_status;

  // Timeout guard (replaces example's fixed delay(ROUND_DELAY))
  if (tag_timeout_ > TAG_TIMEOUT_MS) {
    Serial.printf("[UWB] IB timeout: peer=%d stage=%d\n",
                  peer_beacons_[current_peer_index_], inter_beacon_stage_);
    Serial.flush();
    peer_ranges_[current_peer_index_].peer_id =
        peer_beacons_[current_peer_index_];
    peer_ranges_[current_peer_index_].valid = false;
    peer_ranges_[current_peer_index_].error_code = 1;
    current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
    inter_beacon_stage_ = 0;
    DW3000.standardRX();
    anchor_stage_ = 0;
    return;
  }

  switch (inter_beacon_stage_) {
    case 1:
      // example case 0: send frame 1
      t_roundA_ = 0;
      t_replyA_ = 0;
      DW3000.ds_sendFrame(1);
      tag_tx_ = DW3000.readTXTimestamp();
      inter_beacon_stage_ = 2;
      break;

    case 2:
      // example case 1: await first response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            inter_beacon_stage_ = 0;
            current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
            DW3000.standardRX();
            anchor_stage_ = 0;
          } else if (DW3000.ds_getStage() != 2) {
            DW3000.ds_sendErrorFrame();
            inter_beacon_stage_ = 0;
            current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
            DW3000.standardRX();
            anchor_stage_ = 0;
          } else {
            inter_beacon_stage_ = 3;
          }
        } else {
          DW3000.clearSystemStatus();
          inter_beacon_stage_ = 0;
          current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
          DW3000.standardRX();
          anchor_stage_ = 0;
        }
      }
      break;

    case 3:
      // example case 2: send frame 3
      tag_rx_ = DW3000.readRXTimestamp();
      DW3000.ds_sendFrame(3);
      t_roundA_ = tag_rx_ - tag_tx_;
      tag_tx_ = DW3000.readTXTimestamp();
      t_replyA_ = tag_tx_ - tag_rx_;
      inter_beacon_stage_ = 4;
      break;

    case 4:
      // example case 3: await final response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            inter_beacon_stage_ = 0;
            current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
            DW3000.standardRX();
            anchor_stage_ = 0;
          } else {
            clock_offset_ = DW3000.getRawClockOffset();
            inter_beacon_stage_ = 5;
          }
        } else {
          DW3000.clearSystemStatus();
          inter_beacon_stage_ = 0;
          current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
          DW3000.standardRX();
          anchor_stage_ = 0;
        }
      }
      break;

    case 5: {
      // example case 4: calculate distance
      int ranging_time =
          DW3000.ds_processRTInfo(t_roundA_, t_replyA_, DW3000.read(0x12, 0x04),
                                  DW3000.read(0x12, 0x08), clock_offset_);
      float distance = DW3000.convertToCM(ranging_time);

      UWBRangeMeasurement& r = peer_ranges_[current_peer_index_];
      r.peer_id = peer_beacons_[current_peer_index_];
      r.distance_cm = distance;
      r.clock_offset = clock_offset_;
      r.tx_timestamp = tag_tx_;
      r.rx_timestamp = tag_rx_;
      r.valid = true;
      r.error_code = 0;
      num_peer_ranges_++;
      data_.ranging_count++;
      Serial.printf("[UWB] IB range: peer=%d dist=%.1fcm\n", r.peer_id,
                    distance);
      Serial.flush();

      current_peer_index_ = (current_peer_index_ + 1) % num_peer_beacons_;
      inter_beacon_stage_ = 0;
      DW3000.standardRX();
      anchor_stage_ = 0;
      break;
    }

    default:
      inter_beacon_stage_ = 0;
      DW3000.standardRX();
      anchor_stage_ = 0;
      break;
  }
}

void UWBDriver::handleError(const char* msg) {
  (void)msg;
  data_.error_count++;
  DW3000.clearSystemStatus();
}

void UWBDriver::dw3000ForceIdle() {
  // DW3000 fast command 0x00 (Force to IDLE): byte = 0x81
  // Required before inter-beacon TX because standardRX() leaves the chip in
  // RX mode and TXInstantRX() (0x0C) needs IDLE state to succeed.
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CHIP_SELECT_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(0x81);
  delayMicroseconds(1);
  digitalWrite(CHIP_SELECT_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(10);
}

float UWBDriver::getTemperature() { return DW3000.getTempInC(); }

}  // namespace Drivers
