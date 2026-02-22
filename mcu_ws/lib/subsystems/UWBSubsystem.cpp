/**
 * @file UWBSubsystem.cpp
 * @brief Implementation of UWB subsystem
 * @author SEC26 Team
 * @date 12/22/2025
 */

#include "UWBSubsystem.h"

namespace Subsystem {

bool UWBSubsystem::init() {
  if (!setup_.driver) {
    return false;
  }

  // Initialize the UWB driver
  if (!setup_.driver->init()) {
    Serial.println("[UWB] driver init FAILED");
    Serial.flush();
    return false;
  }

  // Detect mode: tags publish ranging arrays, anchors with peers publish per-pair ranges
  const auto& driver_data = setup_.driver->getData();
  is_tag_mode_ = (driver_data.mode == Drivers::UWBMode::TAG);
  has_peer_ranging_ = !is_tag_mode_ && setup_.driver->hasPeers();

  Serial.printf("[UWB] init ok: tag=%d has_peer_ranging=%d id=%d\n",
                is_tag_mode_, has_peer_ranging_, driver_data.device_id);
  Serial.flush();
  return true;
}

void UWBSubsystem::begin() {
  if (setup_.driver) {
    setup_.driver->begin();
  }
  ranging_timer_ = 0;
}

void UWBSubsystem::update() {
  if (!setup_.driver) {
    return;
  }

  // Update the driver
  setup_.driver->update();

  // If in tag mode, initiate ranging periodically
  if (is_tag_mode_ && ranging_timer_ >= RANGING_INTERVAL_MS) {
    setup_.driver->startRanging();
    ranging_timer_ = 0;
  }

  // Publish ranging data if in tag mode
  if (is_tag_mode_ && pub_.impl) {
    if (everyMs(PUBLISH_INTERVAL_MS)) {
      publishRanging();
    }
  }

  // Publish inter-beacon peer ranges at 2 Hz
  if (has_peer_ranging_) {
    if (everyMs(PEER_PUBLISH_INTERVAL_MS, 1)) {
      publishPeerRanges();
    }
  }
}

void UWBSubsystem::pause() {
  if (setup_.driver) {
    setup_.driver->pause();
  }
}

void UWBSubsystem::reset() {
  if (setup_.driver) {
    setup_.driver->reset();
  }
  pause();
}

bool UWBSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  Serial.printf("[UWB] onCreate: tag=%d peer_ranging=%d\n", is_tag_mode_,
                has_peer_ranging_);
  Serial.flush();

  // Pure anchors (no peers) don't publish anything
  if (!is_tag_mode_ && !has_peer_ranging_) {
    Serial.println("[UWB] pure anchor — no publishers needed");
    Serial.flush();
    return true;
  }

  // TAG mode: create UWBRanging publisher (array of ranges to multiple anchors)
  if (is_tag_mode_) {
    mcu_msgs__msg__UWBRanging__init(&msg_);

    msg_.ranges.capacity = 4;
    msg_.ranges.size = 0;
    msg_.ranges.data = (mcu_msgs__msg__UWBRange*)malloc(
        msg_.ranges.capacity * sizeof(mcu_msgs__msg__UWBRange));

    if (!msg_.ranges.data) {
      return false;
    }

    for (size_t i = 0; i < msg_.ranges.capacity; i++) {
      mcu_msgs__msg__UWBRange__init(&msg_.ranges.data[i]);
    }

    if (rclc_publisher_init_best_effort(
            &pub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, UWBRanging),
            "mcu_uwb/ranging") != RCL_RET_OK) {
      free(msg_.ranges.data);
      msg_.ranges.data = nullptr;
      return false;
    }
  }

  // ANCHOR with peers: create one UWBRange publisher per peer pair
  if (has_peer_ranging_) {
    const uint8_t* peer_ids = setup_.driver->getPeerBeaconIds();
    uint8_t num_peers = setup_.driver->getNumPeerBeacons();
    uint8_t my_id = setup_.driver->getData().device_id;
    num_peer_pubs_ = (num_peers < MAX_PEER_PUBS) ? num_peers : MAX_PEER_PUBS;

    for (uint8_t i = 0; i < num_peer_pubs_; i++) {
      mcu_msgs__msg__UWBRange__init(&peer_msgs_[i]);
      snprintf(peer_topic_names_[i], sizeof(peer_topic_names_[i]),
               "mcu_uwb/range_%d_%d", my_id, peer_ids[i]);
      if (rclc_publisher_init_best_effort(
              &peer_pubs_[i], node,
              ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, UWBRange),
              peer_topic_names_[i]) != RCL_RET_OK) {
        Serial.printf("[UWB] peer pub %d (%s) FAILED\n", i, peer_topic_names_[i]);
        Serial.flush();
        return false;
      }
      Serial.printf("[UWB] peer pub %d: %s ok\n", i, peer_topic_names_[i]);
      Serial.flush();
    }
  }

  return true;
}

void UWBSubsystem::onDestroy() {
  // MicrorosManager::destroy_entities() finalizes rcl_node BEFORE calling
  // onDestroy(), so rcl_publisher_fini() must NOT be called here — it would
  // dereference the already-finalized node and crash. Publishers are cleaned
  // up implicitly by the rmw/rcl layer when the node/support are finalized.
  // We only need to clear our impl pointers so the publisher guards in
  // update() / publishPeerRanges() won't try to use stale handles.
  pub_.impl = nullptr;

  // Free ranges array
  if (msg_.ranges.data) {
    free(msg_.ranges.data);
    msg_.ranges.data = nullptr;
  }
  mcu_msgs__msg__UWBRanging__fini(&msg_);

  // Clear peer publisher handles
  for (uint8_t i = 0; i < num_peer_pubs_; i++) {
    peer_pubs_[i].impl = nullptr;
    mcu_msgs__msg__UWBRange__fini(&peer_msgs_[i]);
  }
  num_peer_pubs_ = 0;

  node_ = nullptr;
}

void UWBSubsystem::updateRangingMessage() {
  if (!setup_.driver) {
    return;
  }

  const auto& driver_data = setup_.driver->getData();

  // Update header timestamp
  msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  // Set tag ID
  msg_.tag_id = driver_data.device_id;

  // Set temperature
  msg_.temperature = driver_data.temperature;

  // Copy range measurements
  msg_.ranges.size = 0;
  for (size_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
    const auto& range = driver_data.ranges[i];
    if (range.valid && msg_.ranges.size < msg_.ranges.capacity) {
      auto& msg_range = msg_.ranges.data[msg_.ranges.size];

      // Update header
      msg_range.header.stamp = msg_.header.stamp;

      // Set IDs
      msg_range.tag_id = driver_data.device_id;
      msg_range.anchor_id = range.peer_id;

      // Set measurement data
      msg_range.distance = range.distance_cm;
      msg_range.signal_strength =
          0.0f;  // Not available from DW3000 in this mode
      msg_range.clock_offset = range.clock_offset;
      msg_range.tx_timestamp = range.tx_timestamp;
      msg_range.rx_timestamp = range.rx_timestamp;

      // Set status
      msg_range.valid = range.valid;
      msg_range.error_code = range.error_code;

      msg_.ranges.size++;
    }
  }

  msg_.num_anchors = msg_.ranges.size;
}

void UWBSubsystem::publishPeerRanges() {
  static uint16_t dbg_count = 0;
  if (dbg_count++ % 10 == 0) {
    Serial.printf("[UWB] publishPeerRanges: num_pubs=%d\n", num_peer_pubs_);
    Serial.flush();
  }
  if (num_peer_pubs_ == 0) {
    return;
  }

  const auto& driver_data = setup_.driver->getData();
  const auto* peer_ranges = setup_.driver->getPeerRanges();
  uint32_t now_sec = (uint32_t)(millis() / 1000);
  uint32_t now_nsec = (uint32_t)((millis() % 1000) * 1000000);

  for (uint8_t i = 0; i < num_peer_pubs_; i++) {
    if (!peer_pubs_[i].impl) {
      continue;
    }
    auto& msg = peer_msgs_[i];
    const auto& r = peer_ranges[i];
    msg.header.stamp.sec = (int32_t)now_sec;
    msg.header.stamp.nanosec = now_nsec;
    msg.tag_id = driver_data.device_id;
    msg.anchor_id = r.peer_id;
    msg.distance = r.distance_cm;
    msg.signal_strength = 0.0f;
    msg.clock_offset = r.clock_offset;
    msg.tx_timestamp = r.tx_timestamp;
    msg.rx_timestamp = r.rx_timestamp;
    msg.valid = r.valid;
    msg.error_code = r.error_code;
    (void)rcl_publish(&peer_pubs_[i], &msg, NULL);
  }
}

void UWBSubsystem::publishRanging() {
  if (!pub_.impl || !is_tag_mode_) {
    return;
  }

  updateRangingMessage();
  (void)rcl_publish(&pub_, &msg_, NULL);
}

}  // namespace Subsystem
