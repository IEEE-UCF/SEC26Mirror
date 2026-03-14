# PID Tuning message for drone flight controller
# Each group: kp, ki, kd

# Roll Rate (inner loop)
float32 roll_rate_kp
float32 roll_rate_ki
float32 roll_rate_kd

# Pitch Rate (inner loop)
float32 pitch_rate_kp
float32 pitch_rate_ki
float32 pitch_rate_kd

# Roll Angle (outer loop)
float32 roll_angle_kp
float32 roll_angle_ki
float32 roll_angle_kd

# Pitch Angle (outer loop)
float32 pitch_angle_kp
float32 pitch_angle_ki
float32 pitch_angle_kd

# Yaw Rate
float32 yaw_rate_kp
float32 yaw_rate_ki
float32 yaw_rate_kd

# Altitude
float32 altitude_kp
float32 altitude_ki
float32 altitude_kd

# Altitude Velocity
float32 alt_vel_kp
float32 alt_vel_ki
float32 alt_vel_kd
