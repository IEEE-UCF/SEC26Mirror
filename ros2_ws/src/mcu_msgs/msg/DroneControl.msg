# Drone control message
std_msgs/Header header

# Position of the drone
geometry_msgs/TransformStamped transform  # Pose of the drone (frame_id -> child_frame_id)
geometry_msgs/Twist twist                 # Linear/angular velocity command