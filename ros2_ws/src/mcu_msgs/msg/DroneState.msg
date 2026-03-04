std_msgs/Header header

# Flight states
uint8 INIT=0
uint8 UNARMED=1
uint8 ARMED=2
uint8 LAUNCHING=3
uint8 VELOCITY_CONTROL=4
uint8 LANDING=5
uint8 EMERGENCY_LAND=6
uint8 state

# Task states
uint8 TASK_NONE=0
uint8 TASK_TAKEOFF=1
uint8 TASK_MOVE_TO_EARTH=2
uint8 TASK_TRANSMIT_DATA=3
uint8 TASK_LAND=4
uint8 TASK_COMPLETE=5
uint8 current_task

# Position and attitude
float32 altitude
float32 pos_x
float32 pos_y
float32 roll
float32 pitch
float32 yaw
