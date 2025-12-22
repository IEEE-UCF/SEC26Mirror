# UWB Anchor configuration and status
std_msgs/Header header

# Anchor identifier
uint8 anchor_id  # ID of this anchor

# Known position (if stationary and pre-configured)
float32 x  # X position in meters (0 if unknown)
float32 y  # Y position in meters (0 if unknown)
float32 z  # Z position in meters (0 if unknown)

# Status
bool is_active      # True if anchor is active and responding
float32 temperature # DW3000 chip temperature in Celsius
uint32 last_seen_ms # Milliseconds since last successful range
