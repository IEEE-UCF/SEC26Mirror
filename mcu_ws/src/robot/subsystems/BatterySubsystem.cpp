#include "BatterySubsystem.h"
#include <micro_ros_utilities/type_utilities.h>

namespace Subsystem {

bool BatterySubsystem::init() {
    return setup_.driver_->init();
}

void BatterySubsystem::update() {
    if(everyMs(100)) {
        setup_.driver_->update();
    }
    if (!pub_.impl) return;
    if (everyMs(1000)) {  // Publish every 1 second
        publishData();
    }
}

void BatterySubsystem::reset() {
    pause();
}

const char* BatterySubsystem::getInfo() {
    static const char info[] = "BatterySubsystem";
    return info;
}

bool BatterySubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
    (void)executor;
    node_ = node;
    if (rclc_publisher_init_best_effort(
          &pub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, BatteryHealth),
          "/mcu_robot/battery_health") != RCL_RET_OK) {
        return false;
    }
    return true;
}

void BatterySubsystem::onDestroy() {
    if (pub_.impl) {
        (void)rcl_publisher_fini(&pub_, node_);
    }
    node_ = nullptr;
}

void BatterySubsystem::publishData() {
    if (!pub_.impl || !setup_.driver_) return;
    
    // Get battery data from driver
    msg.voltage = setup_.driver_->getVoltage();
    msg.current = setup_.driver_->getCurrentmA();
    msg.power = setup_.driver_->getPowermW();
    msg.temperature = setup_.driver_->getTemp();
    
    (void)rcl_publish(&pub_, &msg, NULL);
}

}  // namespace Subsystem