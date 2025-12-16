# Command sent to the MCU bridge to transmit an infrared signal to an external device
uint8 command_code  # IR protocol command byte
uint8 address       # device address (if used by the protocol)
bool repeat         # true if this is a held/repeat signal