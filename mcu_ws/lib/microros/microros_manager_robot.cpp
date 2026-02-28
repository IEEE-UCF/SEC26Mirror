#include "microros_manager_robot.h"

#include <Arduino.h>
#include "DebugLog.h"
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <uxr/client/util/time.h>

#ifdef USE_TEENSYTHREADS
Threads::Mutex g_microros_mutex;
#endif

namespace Subsystem {

MicrorosManager* MicrorosManager::s_instance_ = nullptr;

bool MicrorosManager::create_entities() {
  DEBUG_PRINTLN("[uROS] Creating entities...");
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) {
    DEBUG_PRINTLN("[uROS] FAIL: rclc_support_init");
    return false;
  }
  if (rclc_node_init_default(&node_, "robot_manager", "", &support_) !=
      RCL_RET_OK) {
    DEBUG_PRINTLN("[uROS] FAIL: rclc_node_init_default");
    return false;
  }
  executor_ = rclc_executor_get_zero_initialized_executor();
  // Reserve enough executor handles for all subscription/service callbacks.
  // Publishers do not consume handles — only subscriptions, services, and
  // timers do.  16 handles supports the current set of subsystems with room
  // for growth.
  if (rclc_executor_init(&executor_, &support_.context, 24, &allocator_) !=
      RCL_RET_OK) {
    DEBUG_PRINTLN("[uROS] FAIL: rclc_executor_init");
    return false;
  }
  DEBUG_PRINTLN("[uROS] Node + executor created");
  // Let registered participants create their pubs/subs
  for (size_t i = 0; i < participants_count_; ++i) {
    if (participants_[i] && !participants_[i]->onCreate(&node_, &executor_)) {
      last_failed_participant_ = (int)i;
      DEBUG_PRINTF("[uROS] FAIL: participant[%d] onCreate failed\n", (int)i);
      return false;
    }
    DEBUG_PRINTF("[uROS] participant[%d] onCreate OK\n", (int)i);
  }
  last_failed_participant_ = -1;

  // Manager-owned debug publisher (does not consume an executor handle)
  if (rclc_publisher_init_best_effort(
          &debug_pub_, &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "/mcu_robot/debug") != RCL_RET_OK) {
    DEBUG_PRINTLN("[uROS] FAIL: debug publisher init");
    return false;
  }

  DEBUG_PRINTF("[uROS] All %d participants created successfully\n",
               (int)participants_count_);
  return true;
}

void MicrorosManager::destroy_entities() {
  DEBUG_PRINTLN("[uROS] Destroying entities...");
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // Do NOT call rcl_node_fini — it queues a DELETE_PARTICIPANT XRCE message
  // that leaks into the next session's serial stream on reconnect, causing
  // "unknown reference" errors on the agent.  rclc_support_fini closes the
  // session; the agent tears down all entities when the session ends.
  rclc_executor_fini(&executor_);
  rclc_support_fini(&support_);
  node_ = rcl_get_zero_initialized_node();
  executor_ = rclc_executor_get_zero_initialized_executor();
  // Reset manager-owned debug publisher
  debug_pub_ = rcl_get_zero_initialized_publisher();
  if (debug_msg_.data.data) {
    micro_ros_string_utilities_destroy(&debug_msg_.data);
  }
  // Notify participants to clean up
  for (size_t i = 0; i < participants_count_; ++i) {
    if (participants_[i]) participants_[i]->onDestroy();
  }
}

// No manager-owned timer; executor spin is driven from update()

bool MicrorosManager::init() {
  state_ = WAITING_AGENT;
  return true;
}

void MicrorosManager::begin() {
  s_instance_ = this;
  DEBUG_PRINTLN("[uROS] Setting up serial transport...");
  set_microros_transports();
  DEBUG_PRINTLN("[uROS] Transport ready — waiting for agent");
}

void MicrorosManager::update() {
#ifdef USE_TEENSYTHREADS
  Threads::Scope guard(g_microros_mutex);
#else
  std::lock_guard<std::mutex> guard(mutex_);
#endif

  State prev_state = state_;
  switch (state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                      ? AGENT_AVAILABLE
                                      : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state_ == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(500,
                         state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                                      ? AGENT_CONNECTED
                                      : AGENT_DISCONNECTED;);
      if (state_ == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state_ = WAITING_AGENT;
      break;
    default:
      break;
  }
  if (state_ != prev_state) {
    static const char* const state_names[] = {
        "WAITING_AGENT", "AGENT_AVAILABLE", "AGENT_CONNECTED",
        "AGENT_DISCONNECTED"};
    DEBUG_PRINTF("[uROS] %s -> %s\n", state_names[prev_state],
                 state_names[state_]);
  }
}

void MicrorosManager::pause() {
  if (state_ == AGENT_CONNECTED) {
    destroy_entities();
  }
  state_ = WAITING_AGENT;
}

void MicrorosManager::reset() { pause(); }

const char* MicrorosManager::getInfo() {
  static const char info[] = "MicrorosManager";
  return info;
}

void MicrorosManager::registerParticipant(IMicroRosParticipant* participant) {
  if (participants_count_ <
      (sizeof(participants_) / sizeof(participants_[0]))) {
    participants_[participants_count_++] = participant;
  }
}

#ifdef USE_TEENSYTHREADS
void MicrorosManager::taskFunction(void* pvParams) {
  auto* self = static_cast<MicrorosManager*>(pvParams);
  self->begin();
  while (true) {
    self->update();
    threads.delay(10);
  }
}

void MicrorosManager::beginThreaded(uint32_t stackSize, int /*priority*/) {
  threads.addThread(taskFunction, this, stackSize);
}

Threads::Mutex& MicrorosManager::getMutex() { return mutex_; }
#else
std::mutex& MicrorosManager::getMutex() { return mutex_; }
#endif

bool MicrorosManager::isConnected() const { return state_ == AGENT_CONNECTED; }

void MicrorosManager::debugLog(const char* text) {
  if (!debug_pub_.impl || state_ != AGENT_CONNECTED) return;
  debug_msg_.data = micro_ros_string_utilities_set(debug_msg_.data, text);
#ifdef USE_TEENSYTHREADS
  {
    Threads::Scope guard(g_microros_mutex);
    (void)rcl_publish(&debug_pub_, &debug_msg_, NULL);
  }
#else
  (void)rcl_publish(&debug_pub_, &debug_msg_, NULL);
#endif
}

}  // namespace Subsystem
