# Arm/actuator command
uint8 joint_id     # which arm joint/actuator this command targets (0 = base, 1 = elbow, 2 = wrist, etc.)
int16 position     # target position (degrees or encoder ticks)
uint8 speed        # movement speed (0–255)
