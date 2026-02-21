# MCU Intake Bridge (gear-and-rack) command message
# Subscribed by the Teensy intake bridge subsystem

# Command constants
uint8 CMD_STOW=0
uint8 CMD_EXTEND=1
uint8 CMD_RETRACT=2

uint8 command            # Command to execute (one of the constants above)
