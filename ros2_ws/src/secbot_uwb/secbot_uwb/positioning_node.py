#!/usr/bin/env python3
"""
UWB Positioning Node

This node subscribes to UWB ranging data from tags and performs
trilateration/multilateration to calculate 2D/3D positions of tags.

@author SEC26 Team
@date 12/22/2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from mcu_msgs.msg import UWBRanging, UWBAnchorInfo
import numpy as np
from typing import Dict, Tuple, Optional


class UWBPositioningNode(Node):
    """
    Node for processing UWB range measurements and calculating positions.

    Subscribes to:
        - /mcu_uwb/ranging: UWB range measurements from tags
        - /mcu_uwb/anchor_info: Anchor position updates (optional)

    Publishes:
        - /uwb/pose/<tag_id>: Estimated pose for each tag
    """

    def __init__(self):
        super().__init__('uwb_positioning_node')

        # Declare parameters for anchor positions
        self.declare_parameter('anchor_positions', {
            '10': [0.0, 0.0, 0.0],    # Anchor ID 10 at origin
            '11': [3.0, 0.0, 0.0],    # Anchor ID 11 at 3m on X-axis
            '12': [0.0, 3.0, 0.0],    # Anchor ID 12 at 3m on Y-axis
            '13': [3.0, 3.0, 0.0],    # Anchor ID 13 at (3,3)
        })

        # Load anchor positions from parameters
        self.anchor_positions: Dict[int, np.ndarray] = {}
        anchor_params = self.get_parameter('anchor_positions').value
        for anchor_id_str, pos in anchor_params.items():
            anchor_id = int(anchor_id_str)
            self.anchor_positions[anchor_id] = np.array(pos, dtype=float)
            self.get_logger().info(f'Anchor {anchor_id} at position {pos}')

        # Subscribers
        self.ranging_sub = self.create_subscription(
            UWBRanging,
            'mcu_uwb/ranging',
            self.ranging_callback,
            10
        )

        # Publishers for each tag (created dynamically)
        self.pose_publishers: Dict[int, rclpy.publisher.Publisher] = {}

        # Store latest ranges for each tag
        self.tag_ranges: Dict[int, Dict[int, float]] = {}

        self.get_logger().info('UWB Positioning Node started')

    def ranging_callback(self, msg: UWBRanging):
        """
        Process incoming UWB ranging data and calculate position.

        Args:
            msg: UWBRanging message containing ranges to anchors
        """
        tag_id = msg.tag_id

        # Extract valid ranges
        ranges = {}
        for range_msg in msg.ranges:
            if range_msg.valid:
                anchor_id = range_msg.anchor_id
                # Convert cm to meters
                distance_m = range_msg.distance / 100.0
                ranges[anchor_id] = distance_m

        if len(ranges) < 3:
            self.get_logger().warn(
                f'Tag {tag_id}: Insufficient anchors ({len(ranges)}/3 min) for positioning'
            )
            return

        # Store ranges
        self.tag_ranges[tag_id] = ranges

        # Calculate position using trilateration
        position = self.trilaterate(ranges)

        if position is not None:
            # Publish pose
            self.publish_pose(tag_id, position, msg.header.stamp)

    def trilaterate(self, ranges: Dict[int, float]) -> Optional[np.ndarray]:
        """
        Perform trilateration to estimate 2D/3D position.

        Uses least-squares optimization to find the point that best fits
        the distance measurements to known anchor positions.

        Args:
            ranges: Dictionary mapping anchor_id to distance (meters)

        Returns:
            Estimated position as numpy array [x, y, z] or None if failed
        """
        # Build matrices for least squares
        anchor_ids = list(ranges.keys())
        if len(anchor_ids) < 3:
            return None

        # Filter anchors that we have positions for
        valid_anchors = [aid for aid in anchor_ids if aid in self.anchor_positions]
        if len(valid_anchors) < 3:
            self.get_logger().warn(
                f'Insufficient known anchor positions: {len(valid_anchors)}/3'
            )
            return None

        # Use first anchor as reference
        ref_id = valid_anchors[0]
        ref_pos = self.anchor_positions[ref_id]
        ref_dist = ranges[ref_id]

        # Build system of equations: ||p - a_i||^2 = d_i^2
        # Linearize: 2(a_ref - a_i)·p = ||a_ref||^2 - ||a_i||^2 + d_i^2 - d_ref^2
        A = []
        b = []

        for anchor_id in valid_anchors[1:]:
            anchor_pos = self.anchor_positions[anchor_id]
            dist = ranges[anchor_id]

            # Row of A matrix: 2(a_ref - a_i)
            A.append(2 * (ref_pos - anchor_pos))

            # Right-hand side
            b_val = (
                np.dot(ref_pos, ref_pos) - np.dot(anchor_pos, anchor_pos)
                + dist**2 - ref_dist**2
            )
            b.append(b_val)

        A = np.array(A)
        b = np.array(b)

        # Solve using least squares
        try:
            position, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

            # For 2D positioning, set z=0 if we're only using 2D anchors
            if np.allclose(position[2], 0.0, atol=0.01):
                position[2] = 0.0

            return position

        except np.linalg.LinAlgError as e:
            self.get_logger().error(f'Trilateration failed: {e}')
            return None

    def publish_pose(self, tag_id: int, position: np.ndarray, timestamp):
        """
        Publish estimated pose for a tag.

        Args:
            tag_id: ID of the tag
            position: Estimated position [x, y, z]
            timestamp: Time of measurement
        """
        # Create publisher for this tag if not exists
        if tag_id not in self.pose_publishers:
            topic_name = f'uwb/pose/tag_{tag_id}'
            self.pose_publishers[tag_id] = self.create_publisher(
                PoseWithCovarianceStamped,
                topic_name,
                10
            )
            self.get_logger().info(f'Created publisher for tag {tag_id} on {topic_name}')

        # Create pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'map'  # or 'odom' depending on your setup

        # Set position
        pose_msg.pose.pose.position.x = float(position[0])
        pose_msg.pose.pose.position.y = float(position[1])
        pose_msg.pose.pose.position.z = float(position[2])

        # Set orientation (identity quaternion - no orientation from UWB)
        pose_msg.pose.pose.orientation.w = 1.0
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.0

        # Set covariance (simplified - could be improved with GDOP calculation)
        # Diagonal covariance: uncertainty in x, y, z
        covariance = [0.0] * 36
        covariance[0] = 0.1   # var(x)
        covariance[7] = 0.1   # var(y)
        covariance[14] = 0.1  # var(z)
        pose_msg.pose.covariance = covariance

        # Publish
        self.pose_publishers[tag_id].publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UWBPositioningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
