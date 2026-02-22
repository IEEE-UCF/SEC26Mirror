#!/usr/bin/env python3
"""
goal_diagnostic.py — Static goal calculation verifier.

Subscribes to the full vision pipeline and prints a live diagnostic
summary every second, showing exactly what each stage is producing
and whether the goal math is sensible.

Usage (with sim running):
  python3 goal_diagnostic.py

What it verifies:
  1. /odom       — robot world position (should be ~0,0 if static)
  2. /duck_detections — raw pixel detections from detector_node
  3. /goal_pose  — computed world goal from vision_to_goal
  4. Math check  — independently recomputes expected goal and compares
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from secbot_msgs.msg import DuckDetections


# Camera intrinsics (must match vision_to_goal params)
FX   = 528.0
FY   = 528.0
CX   = 320.0
CY   = 240.0
CAM_HEIGHT   = 0.34   # metres (0.30 URDF + 0.04 wheel radius)
CAM_TILT_DEG = 0.0    # degrees (horizontal)
GOAL_STANDOFF = 0.3   # metres


class GoalDiagnostic(Node):
    def __init__(self):
        super().__init__('goal_diagnostic')

        best_effort = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        reliable    = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

        self.robot_x   = None
        self.robot_y   = None
        self.robot_yaw = None
        self.last_detections = None
        self.last_goal = None

        self.create_subscription(Odometry, '/odom', self.on_odom, best_effort)
        self.create_subscription(DuckDetections, '/duck_detections', self.on_detections, reliable)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, reliable)

        self.create_timer(1.0, self.print_report)
        self.get_logger().info('Goal diagnostic running — waiting for data...')

    def on_odom(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = 2.0 * math.atan2(qz, qw)

    def on_detections(self, msg):
        self.last_detections = msg

    def on_goal(self, msg):
        self.last_goal = msg

    def recompute_goal(self, det):
        """Independent reimplementation of convert_vision_to_goal logic."""
        u = float(det.x + det.w * 0.5)
        v = float(det.y + det.h)

        bearing = -math.atan2((u - CX), FX)
        tilt_rad = math.radians(CAM_TILT_DEG)
        vertical_ray = math.atan2((v - CY), FY)
        tot_down = tilt_rad + vertical_ray

        reasons = []
        if tot_down <= math.radians(1.0):
            reasons.append(f'tot_down={math.degrees(tot_down):.1f}deg <= 1deg → rejected')
            return None, reasons

        dist = CAM_HEIGHT / math.tan(tot_down)
        reasons.append(f'v={v:.0f} cy={CY} → ray={math.degrees(vertical_ray):.1f}deg, dist={dist:.2f}m')

        if not math.isfinite(dist) or dist <= 0 or dist > 8.0:
            reasons.append(f'dist={dist:.2f}m out of range (0, 8m] → rejected')
            return None, reasons

        d_goal = max(0.0, dist - GOAL_STANDOFF)
        if d_goal <= 0:
            reasons.append(f'd_goal={d_goal:.2f}m <= 0 → rejected')
            return None, reasons

        if self.robot_x is None:
            reasons.append('no odom yet → cannot compute world goal')
            return None, reasons

        cyaw = math.cos(self.robot_yaw)
        syaw = math.sin(self.robot_yaw)
        x_rel = d_goal * math.cos(bearing)
        y_rel = d_goal * math.sin(bearing)
        goal_x = self.robot_x + (cyaw * x_rel - syaw * y_rel)
        goal_y = self.robot_y + (syaw * x_rel + cyaw * y_rel)

        reasons.append(f'bearing={math.degrees(bearing):.1f}deg, d_goal={d_goal:.2f}m')
        reasons.append(f'robot=({self.robot_x:.2f},{self.robot_y:.2f}) yaw={math.degrees(self.robot_yaw):.1f}deg')
        reasons.append(f'expected goal=({goal_x:.2f}, {goal_y:.2f})')
        return (goal_x, goal_y), reasons

    def print_report(self):
        sep = '─' * 60
        print(f'\n{sep}')
        print('  GOAL DIAGNOSTIC REPORT')
        print(sep)

        # 1. Robot pose
        if self.robot_x is not None:
            print(f'  ODOM  robot=({self.robot_x:.3f}, {self.robot_y:.3f})'
                  f'  yaw={math.degrees(self.robot_yaw):.1f}°')
        else:
            print('  ODOM  ✗ no /odom received')

        # 2. Detections
        if self.last_detections and self.last_detections.detections:
            dets = self.last_detections.detections
            print(f'  DETS  {len(dets)} detection(s):')
            for i, d in enumerate(dets):
                u = d.x + d.w * 0.5
                v_bot = d.y + d.h
                print(f'    [{i}] id={d.id} conf={d.confidence:.1f}%'
                      f'  center=({u:.0f},{d.y + d.h/2:.0f})'
                      f'  bottom_v={v_bot:.0f}'
                      f'  area={d.area:.0f}')
        else:
            print('  DETS  ✗ no /duck_detections received')

        # 3. goal_pose from vision_to_goal
        if self.last_goal:
            gx = self.last_goal.pose.position.x
            gy = self.last_goal.pose.position.y
            print(f'  GOAL  vision_to_goal published → ({gx:.3f}, {gy:.3f})')
        else:
            print('  GOAL  ✗ no /goal_pose received yet')

        # 4. Independent recomputation on best detection
        if self.last_detections and self.last_detections.detections:
            best = max(self.last_detections.detections, key=lambda d: d.area)
            expected, reasons = self.recompute_goal(best)
            print(f'  RECOMPUTE (largest det id={best.id}):')
            for r in reasons:
                print(f'    {r}')
            if expected and self.last_goal:
                gx = self.last_goal.pose.position.x
                gy = self.last_goal.pose.position.y
                dx = abs(expected[0] - gx)
                dy = abs(expected[1] - gy)
                if dx < 0.05 and dy < 0.05:
                    print(f'  ✅ Goal matches recomputed value (Δ={math.hypot(dx,dy):.3f}m)')
                else:
                    print(f'  ⚠️  MISMATCH: vision_to_goal=({gx:.2f},{gy:.2f})'
                          f' vs recomputed=({expected[0]:.2f},{expected[1]:.2f})'
                          f' Δ=({dx:.2f},{dy:.2f})')

        print(sep)


def main():
    rclpy.init()
    node = GoalDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
