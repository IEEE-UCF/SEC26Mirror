# Drive Base Control Message
std_msgs/Header header

# Mini robot states
uint8 PRE_INIT=0
uint8 INIT=1
uint8 INIT_FAIL=2
uint8 INIT_SUCCESS=3
uint8 ARMED=4
uint8 RUNNING=5
uint8 STOPPED=6
uint8 RESET=7
uint8 EMERGENCY_STOP=8
# Current mini robot state code (one of the constants above)
uint8 state

# Task states (one of the constants below)
uint8 TASK_NONE=0
uint8 TASK_ENTER_CRATER=1
uint8 TASK_CIRCLE_CRATER=2
uint8 TASK_EXIT_CRATER=3
uint8 TASK_PARK=4
uint8 TASK_COMPLETE=5
uint8 TASK_BOARD_ROBOT=6
uint8 current_task