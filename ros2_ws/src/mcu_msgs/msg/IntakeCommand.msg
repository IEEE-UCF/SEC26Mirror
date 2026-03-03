# MCU Intake subsystem command message
# Subscribed by the Teensy intake subsystem

# Command constants
uint8 CMD_INITIALIZE=0
uint8 CMD_EXTEND=1
uint8 CMD_RETRACT=2
uint8 CMD_SET_POSITION=3
uint8 CMD_SET_INTAKE_SPEED=4
uint8 CMD_STOP_RAIL=5
uint8 CMD_STOP_INTAKE=6

uint8 command            # Command to execute (one of the constants above)
float32 value            # Position (0-1) for SET_POSITION, speed (-1 to 1) for SET_INTAKE_SPEED
