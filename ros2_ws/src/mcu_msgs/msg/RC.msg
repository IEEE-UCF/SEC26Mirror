# Flysky RC receiver data
std_msgs/Header header

# Raw channel values from the receiver.
# Convention: 10 channels where 0-3 = joysticks, 4-5 = knobs, 6-9 = switches.
# Value range depends on mapping; here we normalize to [-255, 255].
int32[10] channels

# Switch states (derived from channels 6-9 by thresholding)
bool swa  # switch A
bool swb  # switch B
bool swc  # switch C
bool swd  # switch D

# Knobs (raw values, typically from channels 4-5)
int32 knobl  # left knob value
int32 knobr  # right knob value

# Joysticks (raw values, typically from channels 0-3)
int32 lx  # left stick X
int32 ly  # left stick Y
int32 rx  # right stick X
int32 ry  # right stick Y
