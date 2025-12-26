// traj_controller.h

#ifndef TRAJ_CONTROLLER_H
#define TRAJ_CONTROLLER_H

#include <cmath>
#include <cstddef>
#include <cstdint>

/**
 * @file traj_controller.h
 * @author Rafeed Khan
 * @brief Lightweight trajectory (waypoint) follower for MCU drive bases.
 * No dynamic allocation, deterministic O(1) per update (amortized constant
 * along a path).
 *
 * This is a simple Pure-Pursuit-style “carrot” tracker:
 *  - You give it an array of waypoints (x,y) (optionally final heading /
 * per-point velocity).
 *  - Each update() returns chassis commands (v, w) to drive toward a lookahead
 * point.
 *
 * Intended use:
 *  - The higher-level code converts (v,w) to wheel targets, then uses wheel
 * PID.
 */

class TrajectoryController {
 public:
  struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;  // radians
  };

  struct Waypoint {
    float x = 0.0f;
    float y = 0.0f;

    // Optional per-waypoint desired linear velocity (m/s)
    // If has_vel == 0, we use cfg.cruise_v instead
    float v = 0.0f;
    uint8_t has_vel = 0;

    // Optional final heading, only really matters for last waypoint
    float heading = 0.0f;  // radians
    uint8_t has_heading = 0;
  };

  struct Command {
    float v = 0.0f;  // linear velocity (m/s)
    float w = 0.0f;  // angular velocity (rad/s)

    // Debug helpers (world frame)
    float lookahead_x = 0.0f;
    float lookahead_y = 0.0f;

    bool finished = false;
  };

  struct Config {
    // Pure pursuit lookahead distance (meters)
    // Bigger = smoother but cuts corners
    float lookahead_dist = 0.30f;

    // Cruise speed if waypoints dont specify v
    float cruise_v = 0.50f;

    // Output clamps
    float max_v = 1.0f;
    float max_w = 3.0f;

    // If w saturates, scale down v to keep same curvature (prevents understeer)
    bool preserve_curvature_on_w_saturation = true;

    // Slow down as we approach the final point (linear scale!!)
    float slowdown_dist = 0.50f;  // meters, 0 disables slowdown
    float min_v_near_goal =
        0.0f;  // small creep speed, if we want we can keep it 0 for full stop

    // Goal tolerances
    float pos_tol = 0.03f;      // meters
    float heading_tol = 0.08f;  // radians

    // Final heading behavior (rotate-in-place at the end)
    bool use_final_heading = true;
    float k_heading = 2.5f;  // P gain for final heading hold

    // “Progress” logic: how close before we advance along the path
    float advance_tol = 0.10f;  // meters

    // dt guards
    float min_dt = 1e-6f;
    float max_dt = 0.25f;
  };

  TrajectoryController() = default;
  explicit TrajectoryController(const Config& cfg) { configure(cfg); }

  void configure(const Config& cfg);

  const Config& config() const { return cfg_; }

  // We do NOT copy!!! We just borrow the pointer (no heap)
  void setTrajectory(const Waypoint* pts, size_t count);

  void clearTrajectory();

  bool hasTrajectory() const { return (pts_ != nullptr) && (count_ > 0); }

  void reset();

  // Main update: returns chassis command (v,w) for this timestep
  Command update(const Pose2D& pose, float dt);

  bool isFinished() const { return finished_; }

  // Utility: convert chassis (v,w) to wheel linear speeds (m/s) for diff drive
  // left = v - w*(track_width/2), right = v + w*(track_width/2)
  static void chassisToWheelSpeeds(float v, float w, float track_width,
                                   float* left, float* right);

 private:
  // Finds a lookahead point along the polyline starting at current segment
  // index This is a standard “walk forward along segments by lookahead
  // distance” method
  void computeLookaheadPoint(const Pose2D& pose, float* lx, float* ly);

  // Advances internal segment index when we get close enough to the next
  // waypoint
  void advanceIfNeeded(const Pose2D& pose);

  static float clamp(float x, float lo, float hi);
  static bool isFinite(float x);
  static float wrapAngle(float a);
  static float hypot2(float x, float y);
  static float sanitizePositive(float x, float fallback);

  Config cfg_{};

  // Borrowed, not owned
  const Waypoint* pts_ = nullptr;
  size_t count_ = 0;

  // Path progress
  size_t seg_i_ = 0;  // current segment start index (seg is pts[i] -> pts[i+1])
  bool finished_ = true;

  // Cached lookahead (debug)
  float lookahead_x_ = 0.0f;
  float lookahead_y_ = 0.0f;
};

#endif  // TRAJ_CONTROLLER_H
