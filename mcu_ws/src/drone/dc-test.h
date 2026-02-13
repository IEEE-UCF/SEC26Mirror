#include <pid_controller.h>
#include <stdio.h>
//#include "BNO085.h"
namespace Subsystem{
//     class DroneControlSetup : public Classes ::BaseSetup {
//  public:
//   // direct access to the BNO085 might be necessary...not sure why....
//   DroneControlSetup(const char* _id, Drivers::BNO085Driver* driver)
//       : Classes::BaseSetup(_id), driver_(driver) {}
//   Drivers::BNO085Driver* driver_ = nullptr;
// };
//     class DroneControlSubsystem :public Subsystem::TimedSubSystem{
//         public:
//           explicit DroneControlSubsystem(const DroneControlSetup& setup)
//             : Subsystem::TimedSubSystem(setup), setup_(setup) {}
//         bool init() override;
//         void update() override;
//         private:
//         const DroneControlSetup setup_oject;

//all of this will be simulated through the terminal
//pretend you are using an arduino
        //simulating with the notions of JUST the PID Controller
        //in its place will be the actual msgs and whatnot coming from the PI
        PIDController yaw_sim;
        PIDController roll_sim;
        PIDController pitch_sim;
        typedef struct{
            float roll = 0.0f;
            float yaw = 0.0f;
            float pitch = 0.0f;
            float base_throttle = 0.0f; //will replace with 
        }pi_target_value;
        typedef struct{
            float roll = 0.0f;
            float yaw = 0.0f;
            float pitch = 0.0f;
            float base_throttle = 0.0f; //will replace with 
        }esp_current_value;//will simulate randomization sensor data might be better however!
        typedef enum{
            TILT_LEFT,
            TILT_RIGHT,
            TILT_FRONT,
            TILT_BACK,
            STATIONARY
        }DRONE_STATE;
        typedef struct{
            float ki=0.0f;
            float kp=0.0f;
            float kd=0.0f;
        }differential_gains;
        typedef struct{
            float min_pwm = 0.0f;
            float max_pwm = 255.0f;
        }motor_limits;
        void init();//we will simulate turning on the motors...
        void begin();
        const int MOTOR_FRONT_LEFT = 12;
        const int MOTOR_FRONT_RIGHT = 13;
        const int MOTOR_BACK_LEFT = 14;
        const int MOTOR_BACK_RIGHT = 15;
        //};
}