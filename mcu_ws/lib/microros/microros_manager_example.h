/**
 * @file microros_manager.h
 * @brief Defines the universal microros manager class
 * @author Aldem Pido
 * @date 12/12/2025
 */

#ifndef MICROROS_MANAGER_H
#define MICROROS_MANAGER_H
#include <BaseSubsystem.h>
// microros includes
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>
#include "microros_setup.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do {                                 \
    static volatile int64_t init = -1; \
    if (init == -1) {                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS) {    \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

namespace Subsystem {
class MicrorosManagerSetup : public Classes::BaseSetup {
 public:
  MicrorosManagerSetup(const char* _id) : Classes::BaseSetup(_id) {};
};
class MicrorosManager : public Classes::BaseSubsystem {
 public:
  ~MicrorosManager() override = default;
  MicrorosManager(const MicrorosManagerSetup& setup)
      : BaseSubsystem(setup), setup_(setup) {};
  
  bool init() override;
  void update() override;
  void begin() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;
 
 private:
  const MicrorosManagerSetup setup_;
  rclc_support_t support_;
  rcl_node_t node_;
  rcl_timer_t timer_;
  rclc_executor_t executor_;
  rcl_allocator_t allocator_;
  rcl_publisher_t publisher_;
  std_msgs__msg__Int32 msg_;

  enum State {
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
  } state_;

  static MicrorosManager* s_instance_;
  static void timer_callback(rcl_timer_t* timer, int64_t last_call_time);
  bool create_entities();
  void destroy_entities();
};

}  // namespace Subsystem

#endif
