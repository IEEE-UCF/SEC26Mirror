# Collection of UWB range measurements from a tag to multiple anchors
std_msgs/Header header

# Tag identifier
uint8 tag_id  # ID of the tag device reporting these ranges

# Array of range measurements
UWBRange[] ranges  # Ranges to all anchors

# Device status
float32 temperature  # DW3000 chip temperature in Celsius
uint8 num_anchors    # Number of anchors successfully ranged
