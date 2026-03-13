#include "DroneUWBSubsystem.h"

namespace Drone {

void DroneUWBSubsystem::update() {
  uwb_.update();
  uwb_.startRanging();

  // Feed valid ranges to EKF (per-range scalar updates)
  const auto& data = uwb_.getData();
  float distances[Drivers::UWBDriverData::MAX_ANCHORS];
  uint8_t ids[Drivers::UWBDriverData::MAX_ANCHORS];
  uint8_t n = 0;
  for (uint8_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
    if (data.ranges[i].valid) {
      distances[n] = data.ranges[i].distance_cm / 100.0f;
      ids[n] = data.ranges[i].peer_id;
      n++;
    }
  }
  if (n > 0) {
    ekf_.updateUWB(distances, ids, n);
  }

  // Populate message for deferred publishing
  populateMsg();
}

void DroneUWBSubsystem::populateMsg() {
  if (!pub_.impl) return;

  const auto& data = uwb_.getData();

  if (data_mutex_) xSemaphoreTake(data_mutex_, portMAX_DELAY);

  msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
  msg_.tag_id = data.device_id;
  msg_.temperature = data.temperature;

  msg_.ranges.size = 0;
  for (size_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
    const auto& range = data.ranges[i];
    if (range.valid && msg_.ranges.size < msg_.ranges.capacity) {
      auto& mr = msg_.ranges.data[msg_.ranges.size];
      mr.header.stamp = msg_.header.stamp;
      mr.tag_id = data.device_id;
      mr.anchor_id = range.peer_id;
      mr.distance = range.distance_cm;
      mr.signal_strength = 0.0f;
      mr.clock_offset = range.clock_offset;
      mr.tx_timestamp = range.tx_timestamp;
      mr.rx_timestamp = range.rx_timestamp;
      mr.valid = range.valid;
      mr.error_code = range.error_code;
      msg_.ranges.size++;
    }
  }
  msg_.num_anchors = msg_.ranges.size;
  data_ready_ = true;

  if (data_mutex_) xSemaphoreGive(data_mutex_);
}

}  // namespace Drone
