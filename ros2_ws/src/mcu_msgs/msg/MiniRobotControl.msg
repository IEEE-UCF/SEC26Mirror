# Drive Base Control Message
std_msgs/Header header

# Position of the robot
geometry_msgs/TransformStamped transform  # Pose of the robot (frame_id -> child_frame_id)
geometry_msgs/Twist twist                 # Linear/angular velocity command