#include <DroneControlSubsystem.h>
namespace Subsystem{
    bool DroneControlSubsystem::onCreate(rcl_node_t *usernode, rclc_executor_t *userexecutor){
        drone_control_node = usernode;
        drone_control_executor = userexecutor;

        rcl_allocator_t allocator = rcl_get_default_allocator();//I guess standard memory management for the allocation of a structure
        mcu_msgs_msg__DroneControl(&drone_control_msg);
        //kinda like the equivalent of ESP_OK
        if(rclc_publisher_init_best_effort(&drone_control_publisher,drone_control_node,ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs,msg,DroneControl), "drone_control/status")!=RCL_RET_OK){
            return false; //could not make publisher
        }
        if(rclc_subscription_init_best_effort(&drone_control_subscriber,drone_control_node, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs,msg,DroneControl),"drone_control/commands")!=RCL_RET_OK){
            return false; //could nto make subscriber
        }
        if(rclc_executor_add_subscription_with_context(userexecutor,&drone_control_subscriber, &drone_control_msg, &DroneControlSubsystem::drone_control_callback,this,ON_NEW_DATA)!=RCL_RET_OK){
            return false;//could not make executor<--need this to check the msgs from DroneState msg
        }
        return true;
    }
    void DroneControlSubsystem::onDestroy(){//the equivalent of freeing allocated memory of the heap....maybe...
        rcl_publisher_fini(&drone_control_publisher,drone_control_node);
        rcl_subscription_fini(&drone_control_subscriber,drone_control_node);
        mcu_msgs_msg__DroneControl__fini(&drone_control_msg);
        drone_control_node = nullptr;
    }
    bool DroneControlSubsystem::init(){

        //will probably need to initialize motors..maybe even IR state....maybe..need to think more on this
    }
    void DroneControlSubsystem::update(){

    }
    void DroneControlSubsystem::begin(){
        update();
    }
    static void DroneControlSubsystem::drone_control_callback(const void * msvin, void *context){//check the Dronestate.msg
        DroneControlSubsystem *instance = (DroneControlSubsystem*)context;
        const mcu_msgs_msg__DroneControl *msg_receive = (const mcu_msgs_msg__DroneControl*) msvin;
         
    }
}