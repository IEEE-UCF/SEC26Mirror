#include "dc-test.h"
namespace Subsystem{
    void init(){
        printf("setting pins to pwm...\n");
        //pinMode(MOTOR_FRONT_LEFT,OUTPUT);
        //pinMode(MOTOR_FRONT_RIGHT,OUTPUT);
        //ETC...
        printf("motors set!\n");
        printf("intializing PID angle instances!\n");
        yaw_sim.setGains(1.5,0.0,.1);
        roll_sim.setGains(1.5,0.0,.1);
        pitch_sim.setGains(1.5,0.0,.1);
        

    }

}