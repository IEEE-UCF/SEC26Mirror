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

// IRAM_ATTR is ESP32-specific; define as no-op on other platforms
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

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
                 uint8_t cs_pin, uint8_t rst_pin = 255, uint8_t irq_pin = 255)
      : Classes::BaseSetup(_id),
        mode(mode),
        device_id(device_id),
        cs_pin(cs_pin),
        rst_pin(rst_pin),
        irq_pin(irq_pin) {}

  const UWBMode mode;       // Operating mode (tag or anchor)
  const uint8_t device_id;  // Unique device ID (0-255)
  const uint8_t cs_pin;     // SPI chip select pin
  const uint8_t rst_pin;    // Reset pin (255 = not used)
  const uint8_t irq_pin;    // IRQ pin (255 = not connected, uses polling)
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

  // Inter-beacon peer ranging (call after init(), before begin())
  void setPeerBeacons(const uint8_t* peer_ids, uint8_t count,
                      uint32_t interval_ms = 500);
  bool hasPeers() const { return num_peer_beacons_ > 0; }
  const uint8_t* getPeerBeaconIds() const { return peer_beacons_; }
  uint8_t getNumPeerBeacons() const { return num_peer_beacons_; }
  const UWBRangeMeasurement* getPeerRanges() const { return peer_ranges_; }
  uint8_t getNumPeerRanges() const { return num_peer_ranges_; }

  // Common methods
  float getTemperature();
  bool isInitialized() const { return data_.initialized; }

  // Power management helpers (used by BeaconLogic for light sleep decisions)
  // True when anchor is idle — not mid-exchange with robot or a peer
  bool isIdle() const {
    return data_.mode == UWBMode::ANCHOR && anchor_stage_ == 0 &&
           inter_beacon_stage_ == 0;
  }
  // Milliseconds until the next scheduled inter-beacon ranging event.
  // Returns a large value when there are no peers (sleep indefinitely, wake on
  // IRQ).
  uint32_t timeUntilNextEventMs() const {
    if (num_peer_beacons_ == 0) return 60000;
    uint32_t elapsed = (uint32_t)inter_beacon_timer_;
    if (elapsed >= inter_beacon_interval_ms_) return 0;
    return inter_beacon_interval_ms_ - elapsed;
  }

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

  // Inter-beacon peer ranging (ANCHOR mode only — lower ID initiates)
  uint8_t peer_beacons_[UWBDriverData::MAX_ANCHORS] = {};
  uint8_t num_peer_beacons_ = 0;
  uint8_t current_peer_index_ = 0;
  int inter_beacon_stage_ = 0;  // 0=inactive, 1-5 = TAG exchange stages
  UWBRangeMeasurement peer_ranges_[UWBDriverData::MAX_ANCHORS];
  uint8_t num_peer_ranges_ = 0;
  elapsedMillis inter_beacon_timer_;
  uint32_t inter_beacon_interval_ms_ = 500;  // 2 Hz default

  // Helper methods
  void updateTagMode();
  void updateAnchorMode();
  void processTagStage();
  void processAnchorStage();
  void processInterBeaconStage();
  void handleError(const char* msg);

  // Force DW3000 to IDLE by sending fast command 0x00 (byte 0x81) directly
  // via SPI. Required before TX when the chip is in standardRX() mode.
  // The library's writeFastCommand() is private, so we send it ourselves.
  void dw3000ForceIdle();
};

}  // namespace Drivers

// Global IRQ flag set by hardware ISR — checked by the driver before SPI reads
// and by BeaconLogic for light-sleep decisions.
extern volatile bool g_uwb_irq_fired;
void IRAM_ATTR uwb_irq_handler();

#endif  // UWBDRIVER_H
