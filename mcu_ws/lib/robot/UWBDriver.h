/**
 * @file UWBDriver.h
 * @author SEC26 Team
 * @brief Driver for DW3000 UWB module with tag/anchor modes
 * @date 12/22/2025
 */

#ifndef UWBDRIVER_H
#define UWBDRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <DW3000.h>
#include <elapsedMillis.h>

namespace Drivers {

// UWB operating modes
enum class UWBMode : uint8_t {
  TAG = 0,    // Initiator - requests ranging from anchors
  ANCHOR = 1  // Responder - responds to ranging requests
};

// Range measurement structure
struct UWBRangeMeasurement {
  uint8_t peer_id = 0;        // ID of the peer device
  float distance_cm = 0.0f;   // Distance in centimeters
  int32_t clock_offset = 0;   // Raw clock offset
  uint64_t tx_timestamp = 0;  // TX timestamp
  uint64_t rx_timestamp = 0;  // RX timestamp
  bool valid = false;         // True if measurement is valid
  uint8_t error_code = 0;     // Error code (0 = no error)
};

// UWB driver data
struct UWBDriverData {
  UWBMode mode = UWBMode::TAG;
  uint8_t device_id = 0;
  float temperature = 0.0f;

  // For TAG mode: store ranges to multiple anchors
  static constexpr size_t MAX_ANCHORS = 4;
  UWBRangeMeasurement ranges[MAX_ANCHORS];
  uint8_t num_valid_ranges = 0;

  // For ANCHOR mode: last ranging info
  UWBRangeMeasurement last_range;

  // Status
  bool initialized = false;
  uint32_t ranging_count = 0;
  uint32_t error_count = 0;
};

// UWB driver setup
class UWBDriverSetup : public Classes::BaseSetup {
 public:
  ~UWBDriverSetup() = default;
  UWBDriverSetup() = delete;

  UWBDriverSetup(const char* _id, UWBMode mode, uint8_t device_id,
                 uint8_t cs_pin, uint8_t rst_pin = 255)
      : Classes::BaseSetup(_id),
        mode(mode),
        device_id(device_id),
        cs_pin(cs_pin),
        rst_pin(rst_pin) {}

  const UWBMode mode;       // Operating mode (tag or anchor)
  const uint8_t device_id;  // Unique device ID (0-255)
  const uint8_t cs_pin;     // SPI chip select pin
  const uint8_t rst_pin;    // Reset pin (255 = not used)
};

// UWB driver class
class UWBDriver : public Classes::BaseDriver {
 public:
  UWBDriver(const UWBDriverSetup& setup) : BaseDriver(setup), setup_(setup) {}

  bool init();
  void begin();
  void update();
  void pause();
  void reset();
  const char* getInfo();

  // Get current data
  const UWBDriverData& getData() const { return data_; }
  UWBDriverData& getDataMutable() { return data_; }

  // Tag mode methods
  void setTargetAnchors(const uint8_t* anchor_ids, uint8_t count);
  bool startRanging();  // Initiate ranging to next anchor

  // Anchor mode methods
  bool processRangingRequest();  // Process incoming ranging request

  // Common methods
  float getTemperature();
  bool isInitialized() const { return data_.initialized; }

 private:
  const UWBDriverSetup setup_;
  UWBDriverData data_;

  // TAG mode state
  uint8_t target_anchors_[UWBDriverData::MAX_ANCHORS];
  uint8_t num_target_anchors_ = 0;
  uint8_t current_anchor_index_ = 0;
  int tag_stage_ = 0;
  int t_roundA_ = 0;
  int t_replyA_ = 0;
  uint64_t tag_tx_ = 0;
  uint64_t tag_rx_ = 0;
  int clock_offset_ = 0;
  elapsedMillis tag_timeout_;
  static constexpr uint32_t TAG_TIMEOUT_MS = 100;
  static constexpr uint32_t RANGING_DELAY_MS = 50;

  // ANCHOR mode state
  int anchor_stage_ = 0;
  int t_roundB_ = 0;
  int t_replyB_ = 0;
  uint64_t anchor_tx_ = 0;
  uint64_t anchor_rx_ = 0;

  // Helper methods
  void updateTagMode();
  void updateAnchorMode();
  void processTagStage();
  void processAnchorStage();
  void handleError(const char* msg);
};

}  // namespace Drivers

#endif  // UWBDRIVER_H
