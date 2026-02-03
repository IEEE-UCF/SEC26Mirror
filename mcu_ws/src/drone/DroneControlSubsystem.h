/*
Subsystem to dictate movement for drone(commands from pi to MCU)
no intellisense?
msg: UWB, IMU SENSOR, IR_COMMAND,
*/
#include <BNO085.h>
#include <BaseSubsystem.h>
#include <microros_manager_robot.h>  //any node intialized in any other subsytem will have the "same" ID as the robot..
#include <pid_controller.h>
#include <src/robot/subsystems/ImuSubsystem.h>

#include "TimedSubsystem.h"
namespace Subsystem {
// Setup based on BaseSetup
class DroneControlSetup : public Classes ::BaseSetup {
 public:
  // direct access to the BNO085 might be necessary...not sure why....
  DroneControlSetup(const char* _id, Drivers::BNO085Driver* driver)
      : Classes::BaseSetup(_id), driver_(driver) {}
  Drivers::BNO085Driver* driver_ = nullptr;
};
// MicrosParticipant to communicate to PI, TimedSubSystem for time???
class DroneControlSubsystem : public IMicroRosParticipant,
                              public Subsystem::TimedSubSystem {
  // some constructor needs to be here but for what??
  // basic classes derived from the bAse Class
 public:
  explicit DroneControlSubsystem(const DroneControlSetup& setup)
      : Subsystem::TimedSubSystem(setup), setup_(setup) {}
  bool init() override;  // we might have to initalize with no config struct.
                         // for the PID object for the yaw,pitchm, roll
  void begin() override {};  // going to call the update function
  void update()
      override;  // update function will check very quickly the values from the
                 // sensors and then report back to whoever is listening...not
                 // exactly sure who to report the data to...
  void pause() override {};
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;  //
  void onDestroy() override;                                            //
  void publishData();

 private:
  struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
  };

  struct Limits {
    // Output clamp (command saturation)
    float out_min = -1e9f;
    float out_max = 1e9f;

    // Integral clamp (anti windup)
    float i_min = -1e9f;
    float i_max = 1e9f;
  };
enum class DerivativeMode : uint8_t {
  OnError,       // d(error)/dt (can kick when setpoint steps)
  OnMeasurement  // -d(measurement)/dt (this is more stable for setpoint
                 // steps)
};

typedef struct {
    Gains gain{};//ki,kd,kp
    Limits lim{};//clamps?
    DerivativeMode dmode = DerivativeMode::OnMeasurement; //derror, dmeasurement
    bool conditional_integration = true;
    float d_filter_alpha = 0.0f;
    float min_dt = 1e-6f;
    float max_dt = 1.0f; 
} configuration_settings_for_drone;  // as of now we do not know the actual
                                     // values needed to test drive but we are
                                     // going to have to do trial and run to be
                                     // able to figure those out

static void drone_control_callback(const void* msvin, void* context);
const ImuSubsystemSetup setup;  // object of of our setup we placed up there^^
rcl_node_t* drone_control_node = nullptr;
rcl_publisher_t drone_control_publisher{};  // we are going to send position of
                                            // drone consistently
rclc_executor_t* drone_control_executor =
    nullptr;  // needs to check msgs from pi...for now we assume DroneState.msg
rcl_subscription_t
    drone_control_subscriber;  // if we are to execute with a callback function
                               // we need to subscribe to the pi
mcu_msgs_msg__DroneControl drone_control_msg;  // the message struct??
}
}
