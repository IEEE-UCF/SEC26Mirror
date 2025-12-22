/**
 * @file UWBDriver.cpp
 * @brief Implementation of DW3000 UWB driver
 * @author SEC26 Team
 * @date 12/22/2025
 */

#include "UWBDriver.h"

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
  } else {
    tag_stage_ = 0;
  }
}

void UWBDriver::reset() {
  pause();
  data_.ranging_count = 0;
  data_.error_count = 0;
  data_.num_valid_ranges = 0;
}

const char* UWBDriver::getInfo() {
  static char info[64];
  const char* mode_str = (data_.mode == UWBMode::TAG) ? "TAG" : "ANCHOR";
  snprintf(info, sizeof(info), "UWB %s ID=%d Init=%d", mode_str, data_.device_id,
           data_.initialized);
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

void UWBDriver::updateTagMode() {
  processTagStage();
}

void UWBDriver::processTagStage() {
  int rx_status;

  // Check for timeout
  if (tag_stage_ > 0 && tag_timeout_ > TAG_TIMEOUT_MS) {
    handleError("TAG timeout");
    tag_stage_ = 0;
    current_anchor_index_ = (current_anchor_index_ + 1) % num_target_anchors_;
    return;
  }

  switch (tag_stage_) {
    case 0:
      // Idle - waiting for startRanging() call
      break;

    case 1:
      // Start ranging
      t_roundA_ = 0;
      t_replyA_ = 0;
      DW3000.ds_sendFrame(1);
      tag_tx_ = DW3000.readTXTimestamp();
      tag_stage_ = 2;
      break;

    case 2:
      // Await first response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame() || DW3000.ds_getStage() != 2) {
            handleError("TAG stage 2 error");
            tag_stage_ = 0;
          } else {
            tag_stage_ = 3;
          }
        } else {
          handleError("TAG RX error");
          tag_stage_ = 0;
        }
      }
      break;

    case 3:
      // Response received, send second ranging
      tag_rx_ = DW3000.readRXTimestamp();
      t_roundA_ = tag_rx_ - tag_tx_;
      DW3000.ds_sendFrame(3);
      tag_tx_ = DW3000.readTXTimestamp();
      t_replyA_ = tag_tx_ - tag_rx_;
      tag_stage_ = 4;
      break;

    case 4:
      // Await second response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame()) {
            handleError("TAG stage 4 error");
            tag_stage_ = 0;
          } else {
            clock_offset_ = DW3000.getRawClockOffset();
            tag_stage_ = 5;
          }
        } else {
          handleError("TAG RX error");
          tag_stage_ = 0;
        }
      }
      break;

    case 5: {
      // Calculate distance
      int ranging_time = DW3000.ds_processRTInfo(
          t_roundA_, t_replyA_, DW3000.read(0x12, 0x04), DW3000.read(0x12, 0x08),
          clock_offset_);
      float distance = DW3000.convertToCM(ranging_time);

      // Store result
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

      // Move to next anchor
      current_anchor_index_ = (current_anchor_index_ + 1) % num_target_anchors_;
      tag_stage_ = 0;

      // Small delay before next ranging
      delay(RANGING_DELAY_MS);
      break;
    }

    default:
      tag_stage_ = 0;
      break;
  }
}

void UWBDriver::updateAnchorMode() {
  processAnchorStage();
}

void UWBDriver::processAnchorStage() {
  int rx_status;

  switch (anchor_stage_) {
    case 0:
      // Await ranging request
      t_roundB_ = 0;
      t_replyB_ = 0;

      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame() || DW3000.ds_getStage() != 1) {
            DW3000.ds_sendErrorFrame();
            DW3000.standardRX();
          } else {
            anchor_stage_ = 1;
          }
        } else {
          DW3000.clearSystemStatus();
          DW3000.standardRX();
        }
      }
      break;

    case 1:
      // Ranging received, sending response
      anchor_rx_ = DW3000.readRXTimestamp();
      DW3000.ds_sendFrame(2);
      anchor_tx_ = DW3000.readTXTimestamp();
      t_replyB_ = anchor_tx_ - anchor_rx_;

      // Store tag ID
      data_.last_range.peer_id = DW3000.getSenderID();

      anchor_stage_ = 2;
      break;

    case 2:
      // Awaiting response
      rx_status = DW3000.receivedFrameSucc();
      if (rx_status) {
        DW3000.clearSystemStatus();
        if (rx_status == 1) {
          if (DW3000.ds_isErrorFrame() || DW3000.ds_getStage() != 3) {
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
      // Second response received, sending information frame
      anchor_rx_ = DW3000.readRXTimestamp();
      t_roundB_ = anchor_rx_ - anchor_tx_;
      DW3000.ds_sendRTInfo(t_roundB_, t_replyB_);

      data_.last_range.valid = true;
      data_.ranging_count++;

      // Return to waiting state
      anchor_stage_ = 0;
      DW3000.standardRX();
      break;

    default:
      anchor_stage_ = 0;
      DW3000.standardRX();
      break;
  }
}

void UWBDriver::handleError(const char* msg) {
  (void)msg;  // Can log this if needed
  data_.error_count++;
  DW3000.clearSystemStatus();
}

float UWBDriver::getTemperature() {
  return DW3000.getTempInC();
}

}  // namespace Drivers
