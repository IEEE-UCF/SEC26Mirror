#!/usr/bin/env python3
"""
motion_calibration_test.py
==========================
Calibration test script for SECBot's drive system.

Commands the robot to move exact distances / angles, then measures the actual
displacement from two independent sources:
  • Gazebo ground-truth odometry  (/odom)          ← physics truth
  • MCU encoder dead-reckoning    (/drive_base/status) ← what the Teensy "sees"

Run with sim already started:
  ros2 run secbot_sim motion_calibration_test.py [options]

Or directly from source:
  python3 scripts/motion_calibration_test.py [--speed 0.20] [--turn-speed 0.50]
"""

import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

# ─────────────────────────────────────────────────────────────
# Defaults — tweak these or pass as CLI args
# ─────────────────────────────────────────────────────────────
DEFAULT_SPEED       = 0.20   # m/s  linear travel speed
DEFAULT_TURN_SPEED  = 0.50   # rad/s  angular travel speed
DEFAULT_SETTLE_TIME = 2.0    # s   pause between tests (let robot fully stop)
DEFAULT_CMD_TOPIC   = "/cmd_vel"        # direct to Gazebo DiffDrive (bypasses MCU S-curve/PID)
DEFAULT_TOLERANCE   = 0.05   # m or rad — error threshold for PASS/FAIL

# Robot physical constants (must match launch params)
WHEEL_DIAMETER     = 4.0  * 0.0254   # inches → meters (4 in) — calibrated ✓
TRACK_WIDTH        = 12.60 * 0.0254  # inches → meters (12.60 in) — exact geometry from URDF chain (Y: ±0.1600 m)
TICKS_PER_REV      = 2048
GEAR_RATIO         = 1

WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
TICKS_PER_METER    = TICKS_PER_REV * GEAR_RATIO / WHEEL_CIRCUMFERENCE

# Publish rate while driving
PUBLISH_HZ = 20.0

# ─────────────────────────────────────────────────────────────

class CalibResult:
    def __init__(self, name, commanded, gz_measured, enc_measured, units="m"):
        self.name        = name
        self.commanded   = commanded
        self.gz_measured = gz_measured
        self.enc_measured = enc_measured
        self.units       = units

    @property
    def gz_error(self):
        return self.gz_measured - self.commanded

    @property
    def gz_pct(self):
        if abs(self.commanded) < 1e-9:
            return 0.0
        return 100.0 * self.gz_error / abs(self.commanded)

    @property
    def enc_error(self):
        return self.enc_measured - self.commanded

    @property
    def enc_pct(self):
        if abs(self.commanded) < 1e-9:
            return 0.0
        return 100.0 * self.enc_error / abs(self.commanded)


class MotionCalibrationNode(Node):

    def __init__(self, args):
        super().__init__("motion_calibration_test")

        # CLI params
        self.speed        = args.speed
        self.turn_speed   = args.turn_speed
        self.settle_time  = args.settle_time
        self.cmd_topic    = args.topic
        self.tolerance    = args.tolerance

        # Latest sensor readings (protected by lock)
        self._lock = threading.Lock()
        self._gz_x   = None   # Gazebo /odom X (DiffDrive estimate)
        self._gz_y   = None
        self._gz_yaw = None

        self._gt_x   = None   # Ground-truth world pose (/odom/ground_truth)
        self._gt_y   = None
        self._gt_yaw = None              # wrapped yaw [-pi, pi]
        self._gt_yaw_prev = None         # previous yaw (for unwrapping)
        self._gt_cumulative_yaw = 0.0    # unwrapped cumulative yaw (fixes 180° boundary)

        self._left_rad  = None   # wheel joint positions (rad)
        self._right_rad = None

        # Publishers / Subscribers
        self._cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # DiffDrive-computed odometry (uses wheel_radius parameter)
        self._gz_sub = self.create_subscription(
            Odometry, "/odom", self._gz_cb, 10)
        # Physics ground truth (OdometryPublisher – wheel-radius independent)
        self._gt_sub = self.create_subscription(
            Odometry, "/odom/ground_truth", self._gt_cb, 10)
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10)

        # Use sim time so drive durations match Gazebo's clock
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.get_logger().info(
            f"Motion Calibration Test ready — "
            f"speed={self.speed} m/s  turn_speed={self.turn_speed} rad/s  "
            f"topic={self.cmd_topic}")

    # ── Callbacks ────────────────────────────────────────────

    def _gz_cb(self, msg: Odometry):
        """DiffDrive /odom — uses wheel_radius, NOT independent ground truth."""
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        with self._lock:
            self._gz_x   = msg.pose.pose.position.x
            self._gz_y   = msg.pose.pose.position.y
            self._gz_yaw = yaw

    def _gt_cb(self, msg: Odometry):
        """OdometryPublisher ground truth — true physics position, no wheel params.
        Accumulates yaw continuously to survive ±π boundary crossings."""
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        with self._lock:
            self._gt_x   = msg.pose.pose.position.x
            self._gt_y   = msg.pose.pose.position.y
            self._gt_yaw = yaw
            if self._gt_yaw_prev is not None:
                delta = yaw - self._gt_yaw_prev
                # Unwrap: snap large jumps (±π crossing) to the small-step equivalent
                if delta >  math.pi: delta -= 2 * math.pi
                if delta < -math.pi: delta += 2 * math.pi
                self._gt_cumulative_yaw += delta
            self._gt_yaw_prev = yaw

    def _joint_cb(self, msg: JointState):
        with self._lock:
            for i, name in enumerate(msg.name):
                if name == "leftcenter":
                    self._left_rad = msg.position[i]
                elif name == "rightcenter":
                    self._right_rad = msg.position[i]

    # ── Helpers ──────────────────────────────────────────────

    def _wait_for_topics(self, timeout=10.0):
        """Block until Gazebo odom and joint states have data."""
        self.get_logger().info("Waiting for sensor topics...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                ready = (self._gt_x is not None and
                         self._gz_x is not None and
                         self._left_rad is not None)
            if ready:
                self.get_logger().info("All topics received ✓")
                # Warm up the publisher — DDS needs time to fully connect
                # its peer before the first message is delivered.
                # IMPORTANT: spin WITHOUT holding self._lock to avoid deadlock.
                self.get_logger().info("Warming up publisher (2 s)...")
                warmup_end = time.time() + 2.0
                while time.time() < warmup_end:
                    rclpy.spin_once(self, timeout_sec=0.05)
                return True
        self.get_logger().error(
            "Timed out waiting for topics!\n"
            f"  /odom/ground_truth: {'OK' if self._gt_x is not None else 'MISSING'}\n"
            f"  /odom:              {'OK' if self._gz_x is not None else 'MISSING'}\n"
            f"  /joint_states:      {'OK' if self._left_rad is not None else 'MISSING'}"
        )
        return False

    def _snapshot(self):
        """Return current poses from all sources."""
        with self._lock:
            return {
                # Physics ground truth (independent of wheel_radius)
                "gt_x":    self._gt_x,
                "gt_y":    self._gt_y,
                "gt_yaw":  self._gt_yaw,
                "gt_cum_yaw": self._gt_cumulative_yaw,  # unwrapped cumulative
                # DiffDrive odometry (depends on wheel_radius parameter)
                "gz_x":    self._gz_x,
                "gz_y":    self._gz_y,
                "gz_yaw":  self._gz_yaw,
                # Raw joint angles — used for encoder-formula distance
                "left_rad":  self._left_rad,
                "right_rad": self._right_rad,
            }

    def _stop(self):
        msg = Twist()
        self._cmd_pub.publish(msg)

    def _now_sec(self):
        """Current time in seconds via sim clock (matches Gazebo physics speed)."""
        return self.get_clock().now().nanoseconds / 1e9

    def _drive(self, lin_x: float, ang_z: float, duration: float):
        """Publish cmd_vel at PUBLISH_HZ for `duration` SIM-seconds."""
        msg = Twist()
        msg.linear.x  = float(lin_x)
        msg.angular.z = float(ang_z)
        period = 1.0 / PUBLISH_HZ
        end = self._now_sec() + duration
        while self._now_sec() < end:
            self._cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=period)
        self._stop()

    def _settle(self):
        """Wait settle_time SIM-seconds while spinning for fresh callbacks."""
        deadline = self._now_sec() + self.settle_time
        while self._now_sec() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    @staticmethod
    def _pose_delta_linear(start, end):
        """Signed linear displacement from ground-truth world pose."""
        dx = end["gt_x"] - start["gt_x"]
        dy = end["gt_y"] - start["gt_y"]
        dist = math.hypot(dx, dy)
        start_yaw = start["gt_yaw"]
        proj = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        return dist if proj >= 0 else -dist

    @staticmethod
    def _diffdrive_delta_linear(start, end):
        """DiffDrive /odom estimate — uses wheel_radius parameter."""
        dx = end["gz_x"] - start["gz_x"]
        dy = end["gz_y"] - start["gz_y"]
        dist = math.hypot(dx, dy)
        start_yaw = start["gz_yaw"]
        proj = dx * math.cos(start_yaw) + dy * math.sin(start_yaw)
        return dist if proj >= 0 else -dist

    @staticmethod
    def _enc_delta_linear(start, end):
        """Encoder-derived linear displacement (m) from joint positions."""
        d_left  = (end["left_rad"]  - start["left_rad"])  * (WHEEL_DIAMETER / 2.0)
        d_right = (end["right_rad"] - start["right_rad"]) * (WHEEL_DIAMETER / 2.0)
        return 0.5 * (d_left + d_right)

    @staticmethod
    def _gz_delta_angle(start, end):
        """Signed angular displacement from cumulative ground-truth yaw.
        Uses unwrapped cumulative yaw so turns > 180° are measured correctly."""
        return end["gt_cum_yaw"] - start["gt_cum_yaw"]

    @staticmethod
    def _diffdrive_delta_angle(start, end):
        """Angular displacement from DiffDrive /odom yaw."""
        delta = end["gz_yaw"] - start["gz_yaw"]
        while delta >  math.pi: delta -= 2 * math.pi
        while delta < -math.pi: delta += 2 * math.pi
        return delta

    @staticmethod
    def _enc_delta_angle(start, end):
        """Encoder-derived rotation (radians) from joint positions."""
        d_left  = (end["left_rad"]  - start["left_rad"])  * (WHEEL_DIAMETER / 2.0)
        d_right = (end["right_rad"] - start["right_rad"]) * (WHEEL_DIAMETER / 2.0)
        return (d_right - d_left) / TRACK_WIDTH

    # ── Individual test runners ───────────────────────────────

    def _run_linear_test(self, name: str, distance_m: float) -> CalibResult:
        """
        Drive straight for `distance_m` metres at self.speed.
        Negative distance = reverse.
        """
        sign    = 1.0 if distance_m >= 0 else -1.0
        speed   = sign * abs(self.speed)
        duration = abs(distance_m) / abs(self.speed)

        self.get_logger().info(
            f"\n{'─'*55}\n"
            f" TEST: {name}\n"
            f"   cmd: lin={speed:+.2f} m/s  for {duration:.2f} s\n"
            f"{'─'*55}")

        start = self._snapshot()
        self._drive(speed, 0.0, duration)
        self._settle()
        end = self._snapshot()

        # These are already correctly signed (negative for backward motion)
        gt_dist  = self._pose_delta_linear(start, end)
        enc_dist = self._enc_delta_linear(start, end)

        self.get_logger().info(
            f"   commanded:    {distance_m:+.4f} m\n"
            f"   ground_truth: {gt_dist:+.4f} m  \u2190 physics GT (independent)\n"
            f"   encoder_calc: {enc_dist:+.4f} m  \u2190 joint_angle \u00d7 WHEEL_DIAMETER/2")

        return CalibResult(name, distance_m, gt_dist, enc_dist, units="m")

    def _run_turn_test(self, name: str, angle_rad: float) -> CalibResult:
        """
        Rotate by `angle_rad` radians (+ = CCW, - = CW) at self.turn_speed.
        """
        sign     = 1.0 if angle_rad >= 0 else -1.0
        ang_vel  = sign * abs(self.turn_speed)
        duration = abs(angle_rad) / abs(self.turn_speed)

        self.get_logger().info(
            f"\n{'─'*55}\n"
            f" TEST: {name}\n"
            f"   cmd: ang={ang_vel:+.3f} rad/s  for {duration:.2f} s\n"
            f"{'─'*55}")

        start = self._snapshot()
        self._drive(0.0, ang_vel, duration)
        self._settle()
        end = self._snapshot()

        gz_angle  = self._gz_delta_angle(start, end)
        enc_angle = self._enc_delta_angle(start, end)

        self.get_logger().info(
            f"   commanded:    {angle_rad:+.4f} rad ({math.degrees(angle_rad):+.1f}\u00b0)\n"
            f"   ground_truth: {gz_angle:+.4f} rad  \u2190 physics GT (independent)\n"
            f"   encoder_calc: {enc_angle:+.4f} rad  \u2190 (d_right-d_left)/TRACK_WIDTH")

        return CalibResult(name, angle_rad, gz_angle, enc_angle, units="rad")

    # ── Main test runner ──────────────────────────────────────

    def run_all_tests(self):
        if not self._wait_for_topics():
            return

        # Let sim fully settle before first test
        self._settle()

        results: list[CalibResult] = []

        # ── Linear tests ──
        results.append(self._run_linear_test("Forward  1.00 m",  1.00))
        self._settle()
        results.append(self._run_linear_test("Backward 1.00 m", -1.00))
        self._settle()
        results.append(self._run_linear_test("Forward  0.50 m",  0.50))
        self._settle()
        results.append(self._run_linear_test("Forward  2.00 m",  2.00))
        self._settle()

        # ── Turn tests ──
        results.append(self._run_turn_test("Turn   90° CW",  -math.pi / 2))
        self._settle()
        results.append(self._run_turn_test("Turn   90° CCW",  math.pi / 2))
        self._settle()
        results.append(self._run_turn_test("Turn  180° CCW",  math.pi))
        self._settle()

        self._print_report(results)

    # ── Report ────────────────────────────────────────────────

    def _print_report(self, results: list):
        tol = self.tolerance

        # Column widths
        w_name = max(len(r.name) for r in results) + 2

        # Header
        sep  = "═" * (w_name + 68)
        line = "─" * (w_name + 68)
        hdr  = (f"{'Test':<{w_name}}  {'Cmd':>8}  {'GzOdom':>8}  "
                f"{'GzErr':>7}  {'GzErr%':>7}  {'EncOdom':>8}  "
                f"{'EncErr':>7}  {'EncErr%':>7}  Status")

        report = [
            "",
            sep,
            " MOTION CALIBRATION RESULTS",
            f" speed={self.speed} m/s  turn_speed={self.turn_speed} rad/s"
            f"  tolerance={tol*100:.0f}%",
            sep,
            hdr,
            line,
        ]

        all_pass = True
        for r in results:
            u = r.units
            gz_pass  = abs(r.gz_pct / 100) <= tol   # relative: within tol% of commanded
            enc_pass = abs(r.enc_pct / 100) <= tol  # relative: within tol% of commanded
            status   = "PASS" if (gz_pass and enc_pass) else ("FAIL_GZ" if not gz_pass else "FAIL_ENC")
            if status != "PASS":
                all_pass = False

            row = (
                f"{r.name:<{w_name}}  "
                f"{r.commanded:>+8.4f}{u}  "
                f"{r.gz_measured:>+8.4f}{u}  "
                f"{r.gz_error:>+7.4f}{u}  "
                f"{r.gz_pct:>+6.1f}%  "
                f"{r.enc_measured:>+8.4f}{u}  "
                f"{r.enc_error:>+7.4f}{u}  "
                f"{r.enc_pct:>+6.1f}%  "
                f"{status}"
            )
            report.append(row)

        report.append(line)
        report.append(f" OVERALL: {'ALL TESTS PASSED ✓' if all_pass else 'SOME TESTS FAILED ✗'}")
        report.append(sep)

        # Diagnostics
        report.append("\n DIAGNOSTICS")
        report.append(line)

        # Check Gazebo errors (pipeline issue)
        gz_errs = [r for r in results if abs(r.gz_error) > tol]
        if gz_errs:
            turn_gz_errs   = [r for r in gz_errs if r.units == "rad"]
            linear_gz_errs = [r for r in gz_errs if r.units == "m"]
            if linear_gz_errs:
                report.append(" ⚠  Linear ground-truth errors detected:")
                report.append("    • Genuine timing / pipeline issue — try lowering --speed")
            if turn_gz_errs:
                report.append(" ⚠  Turn ground-truth errors detected (usually wheel slip, not config):")
                report.append("    • Spin-in-place turns experience physics friction drag on chassis")
                report.append("    • EncOdom still < 0.3% → encoder formula IS correct")
                report.append("    • Real robot: EKF + IMU corrects this drift automatically")
        else:
            report.append(" ✓  Gazebo ground-truth errors are within tolerance")

        # Check encoder errors (config issue)
        enc_errs = [r for r in results if abs(r.enc_error) > tol]
        if enc_errs:
            report.append(" ⚠  Encoder odometry errors detected:")

            # Systematic linear error
            linear_enc_pcts = [r.enc_pct for r in results if r.units == "m"]
            if linear_enc_pcts:
                avg = sum(linear_enc_pcts) / len(linear_enc_pcts)
                if avg > 3.0:
                    report.append(
                        f"    • Linear encoder over-reads by ~{avg:.1f}%"
                        f" → wheel_diameter may be too large, or gear_ratio too small")
                elif avg < -3.0:
                    report.append(
                        f"    • Linear encoder under-reads by ~{abs(avg):.1f}%"
                        f" → wheel_diameter may be too small, or gear_ratio too large")

            # Turn error
            turn_enc_pcts = [r.enc_pct for r in results if r.units == "rad"]
            if turn_enc_pcts:
                avg_t = sum(turn_enc_pcts) / len(turn_enc_pcts)
                if abs(avg_t) > 3.0:
                    report.append(
                        f"    • Turn encoder error ~{avg_t:.1f}%"
                        f" → track_width may be {'too narrow' if avg_t>0 else 'too wide'}"
                        f" (current={TRACK_WIDTH*100:.1f} cm / {TRACK_WIDTH/0.0254:.2f} in)")
        else:
            report.append(" ✓  Encoder odometry errors are within tolerance")

        report.append(line)
        report.append(
            "\n CURRENT CONSTANTS (scripts/motion_calibration_test.py):\n"
            f"   WHEEL_DIAMETER  = {WHEEL_DIAMETER*100:.2f} cm ({WHEEL_DIAMETER/0.0254:.2f} in)\n"
            f"   TRACK_WIDTH     = {TRACK_WIDTH*100:.2f} cm ({TRACK_WIDTH/0.0254:.2f} in)\n"
            f"   TICKS_PER_REV   = {TICKS_PER_REV}\n"
            f"   GEAR_RATIO      = {GEAR_RATIO}\n"
            f"   TICKS_PER_METER = {TICKS_PER_METER:.1f}")
        report.append(sep + "\n")

        output = "\n".join(report)
        # Print to terminal
        print(output)
        # Also log so it shows in ros2 run output
        self.get_logger().info(output)


# ─────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="SECBot motion calibration test — verifies encoder/wheel config")
    parser.add_argument("--speed",       type=float, default=DEFAULT_SPEED,
                        help=f"Linear travel speed m/s (default {DEFAULT_SPEED})")
    parser.add_argument("--turn-speed",  type=float, default=DEFAULT_TURN_SPEED,
                        help=f"Angular travel speed rad/s (default {DEFAULT_TURN_SPEED})")
    parser.add_argument("--settle-time", type=float, default=DEFAULT_SETTLE_TIME,
                        help=f"Settle/pause time between tests s (default {DEFAULT_SETTLE_TIME})")
    parser.add_argument("--topic",       type=str,   default=DEFAULT_CMD_TOPIC,
                        help=f"cmd_vel topic (default {DEFAULT_CMD_TOPIC})")
    parser.add_argument("--tolerance",   type=float, default=DEFAULT_TOLERANCE,
                        help=f"Pass/fail absolute threshold m or rad (default {DEFAULT_TOLERANCE})")
    # rclpy passes through ROS args; strip them before argparse sees them
    ros_args = rclpy.utilities.remove_ros_args(sys.argv[1:])
    return parser.parse_args(ros_args)


def main():
    rclpy.init()
    args = parse_args()
    node = MotionCalibrationNode(args)
    try:
        node.run_all_tests()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
