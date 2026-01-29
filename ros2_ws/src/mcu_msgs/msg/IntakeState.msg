# MCU Intake subsystem state message
# Published by the Teensy intake subsystem
std_msgs/Header header

# Intake state machine states
uint8 STATE_IDLE=0
uint8 STATE_SPINNING=1
uint8 STATE_CAPTURED=2
uint8 STATE_JAMMED=3
uint8 STATE_EJECTING=4
uint8 STATE_FAULT=5
uint8 state              # Current state (one of the constants above)

# Duck detection from IR breakbeam
bool duck_detected

# Motor state
uint8 MOTOR_OFF=0
uint8 MOTOR_FORWARD=1
uint8 MOTOR_REVERSE=2
uint8 motor_state        # Current motor state

# Diagnostics
uint32 capture_count     # Total ducks captured this session
uint32 jam_count         # Total jams detected this session
uint32 time_in_state_ms  # Time spent in current state
