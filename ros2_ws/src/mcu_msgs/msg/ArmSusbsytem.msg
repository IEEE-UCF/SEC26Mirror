# MCU Arm subsystem message
# Contains overall arm state, position, and intake status.
std_msgs/Header header

# Arm controller state (lifecycle / health)
uint8 PRE_INIT=0
uint8 INIT=1
uint8 INIT_FAIL=2
uint8 INIT_SUCCESS=3
uint8 ARMED=4
uint8 RUNNING=5
uint8 STOPPED=6
uint8 state            # One of the constants above

# Arm position state (mechanical position)
uint8 POSITION_RETRACTED=0
uint8 POSITION_EXTENDED=1
uint8 POSITION_MOVING=2
uint8 position_state   # One of the constants above

# Intake on/off state
uint8 INTAKE_OFF=0
uint8 INTAKE_ON=1
uint8 intake_state     # One of the constants above

# Intake item detection state
uint8 ITEM_NONE=0
uint8 ITEM_DETECTED=1
uint8 item_state       # One of the constants above

