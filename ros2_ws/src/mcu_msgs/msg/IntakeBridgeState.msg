# MCU Intake Bridge (gear-and-rack) subsystem state message
# Published by the Teensy intake bridge subsystem
std_msgs/Header header

# Intake bridge state machine states
uint8 STATE_STOWED=0
uint8 STATE_EXTENDING=1
uint8 STATE_EXTENDED=2
uint8 STATE_RETRACTING=3
uint8 STATE_ERROR=4
uint8 state              # Current state (one of the constants above)

# Duck detection from TOF sensor
bool duck_detected

# TOF distance reading (mm)
uint16 tof_distance_mm

# Rack motor state
uint8 MOTOR_OFF=0
uint8 MOTOR_EXTENDING=1
uint8 MOTOR_RETRACTING=2
uint8 rack_motor_state   # Current rack motor state

# Diagnostics
uint32 extend_count      # Total extend cycles this session
uint32 retract_count     # Total retract cycles this session
uint32 time_in_state_ms  # Time spent in current state
