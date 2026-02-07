"""
MCU Subsystem Simulator Launch File

Launches the mcu_subsystem_sim node that simulates all MCU subsystems:
Drive (with real S-curve + PID + localization), Battery, Heartbeat,
IMU, TOF, RC, Intake, and MiniRobot.

Drive config defaults match DriveBaseConfig.example.h.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='secbot_sim',
            executable='mcu_subsystem_sim',
            name='mcu_subsystem_sim',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'num_tof_sensors': 4,

                # Drive config (inches, matching DriveBaseConfig.example.h)
                'track_width': 12.0,
                'wheel_diameter': 4.0,
                'encoder_ticks_per_rev': 2048,
                'gear_ratio': 1,
                'max_velocity': 24.0,        # in/s
                'max_angular_velocity': 3.0,  # rad/s
                'max_wheel_velocity': 48.0,   # in/s at PWM=255

                # PID (same for both wheels)
                'pid_kp': 0.8,
                'pid_ki': 0.1,
                'pid_kd': 0.05,
                'pid_out_min': -255.0,
                'pid_out_max': 255.0,
                'pid_i_min': -50.0,
                'pid_i_max': 50.0,
                'pid_d_filter_alpha': 0.1,

                # Linear S-curve motion profile
                'linear_v_max': 24.0,   # in/s
                'linear_a_max': 12.0,   # in/s^2
                'linear_d_max': 12.0,   # in/s^2
                'linear_j_max': 48.0,   # in/s^3

                # Angular S-curve motion profile
                'angular_v_max': 3.0,   # rad/s
                'angular_a_max': 6.0,   # rad/s^2
                'angular_d_max': 6.0,   # rad/s^2
                'angular_j_max': 24.0,  # rad/s^3
            }],
        ),
    ])
