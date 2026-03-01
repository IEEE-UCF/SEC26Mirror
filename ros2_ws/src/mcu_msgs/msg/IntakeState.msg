# MCU Intake subsystem state message
# Published by the Teensy intake subsystem (combined linear rail + spinning motor)
std_msgs/Header header

# Intake state machine states
uint8 STATE_UNINITIALIZED=0
uint8 STATE_CALIBRATING=1
uint8 STATE_IDLE=2
uint8 STATE_MOVING=3
uint8 STATE_FAULT=4
uint8 state              # Current state (one of the constants above)

# Rail position
float32 position         # Normalized 0.0 (retracted) to 1.0 (extended)
float32 setpoint         # Target normalized position

# Encoder data
int32 encoder_ticks      # Raw encoder ticks
int32 min_ticks          # Calibrated retract limit
int32 max_ticks          # Calibrated extend limit

# Limit switches
bool limit_retract_active  # Retract limit switch active
bool limit_extend_active   # Extend limit switch active
bool calibrated

# Spinning intake motor
float32 intake_motor_speed  # Current spinning motor speed (-1.0 to 1.0)

# Diagnostics
uint32 time_in_state_ms  # Time spent in current state
