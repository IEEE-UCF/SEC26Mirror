# Detected antenna state from vision
uint8 antenna_id        # which antenna (1-4 from field layout)
LedColor color          # current LED color on the antenna

uint8 COLOR_UNKNOWN = 0
uint8 COLOR_RED = 1
uint8 COLOR_BLUE = 2
uint8 COLOR_GREEN = 3
uint8 COLOR_PURPLE = 4
string color_class