# Single UWB range measurement
std_msgs/Header header

# Device identifiers
uint8 tag_id        # ID of the tag (initiator) device
uint8 anchor_id     # ID of the anchor (responder) device

# Range measurement
float32 distance    # Distance in centimeters (cm)
float32 signal_strength  # Signal strength/quality indicator
int32 clock_offset  # Raw clock offset between devices

# Timestamps (DW3000 internal timestamps)
uint64 tx_timestamp  # Transmit timestamp from tag
uint64 rx_timestamp  # Receive timestamp at tag

# Status flags
bool valid          # True if this range measurement is valid
uint8 error_code    # Error code (0 = no error)
