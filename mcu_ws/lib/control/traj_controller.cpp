// traj_controller.cpp

/**
 * @file traj_controller.cpp
 * @author Rafeed Khan
 * @brief Trajectory following logic (Pure-Pursuit style) for MCU drive bases.
 * No dynamic allocation, deterministic O(1) update time (amortized along a path).
 */

#include "traj_controller.h"

void TrajectoryController::configure(const Config& cfg) {
  cfg_ = cfg;

  // sanitizing the obvious stuff (keep it safe, no weird negatives)
  cfg_.lookahead_dist = sanitizePositive(cfg_.lookahead_dist, 0.05f);
  cfg_.max_v = sanitizePositive(cfg_.max_v, 0.0f);
  cfg_.max_w = sanitizePositive(cfg_.max_w, 0.0f);

  cfg_.cruise_v = clamp(cfg_.cruise_v, 0.0f, cfg_.max_v);

  cfg_.slowdown_dist = sanitizePositive(cfg_.slowdown_dist, 0.0f);
  cfg_.pos_tol = sanitizePositive(cfg_.pos_tol, 0.0f);
  cfg_.heading_tol = sanitizePositive(cfg_.heading_tol, 0.0f);
  cfg_.advance_tol = sanitizePositive(cfg_.advance_tol, 0.0f);

  cfg_.k_heading = sanitizePositive(cfg_.k_heading, 0.0f);
  cfg_.min_v_near_goal = clamp(cfg_.min_v_near_goal, 0.0f, cfg_.max_v);

  reset();
}

void TrajectoryController::setTrajectory(const Waypoint* pts, size_t count) {
  pts_ = pts;
  count_ = count;

  seg_i_ = 0;
  finished_ = !(pts_ && (count_ > 0));

  lookahead_x_ = 0.0f;
  lookahead_y_ = 0.0f;
}

void TrajectoryController::clearTrajectory() {
  pts_ = nullptr;
  count_ = 0;
  seg_i_ = 0;
  finished_ = true;
}

void TrajectoryController::reset() {
  seg_i_ = 0;
  finished_ = !(pts_ && (count_ > 0));
  lookahead_x_ = 0.0f;
  lookahead_y_ = 0.0f;
}

TrajectoryController::Command TrajectoryController::update(const Pose2D& pose, float dt) {
  Command cmd{};

  // if no path, do nothing
  if (!hasTrajectory()) {
    cmd.finished = true;
    finished_ = true;
    return cmd;
  }

  // reject bad dt (keeps us deterministic and prevents div0 explosions)
  if (!(dt >= cfg_.min_dt) || !(dt <= cfg_.max_dt) || !isFinite(dt)) {
    cmd.v = 0.0f;
    cmd.w = 0.0f;
    cmd.lookahead_x = lookahead_x_;
    cmd.lookahead_y = lookahead_y_;
    cmd.finished = finished_;
    return cmd;
  }

  // handle the degenerate case: single waypoint = “go to point”
  if (count_ == 1) {
    const float dx = pts_[0].x - pose.x;
    const float dy = pts_[0].y - pose.y;
    const float dist = hypot2(dx, dy);

    if (dist <= cfg_.pos_tol) {
      // optional final heading hold, this is just quality of life slop
      float w = 0.0f;
      if (cfg_.use_final_heading && pts_[0].has_heading) {
        const float err_h = wrapAngle(pts_[0].heading - pose.theta);
        if (std::fabs(err_h) > cfg_.heading_tol) {
          w = clamp(cfg_.k_heading * err_h, -cfg_.max_w, cfg_.max_w);
          cmd.v = 0.0f;
          cmd.w = w;
          cmd.finished = false;
          finished_ = false;
          return cmd;
        }
      }

      cmd.v = 0.0f;
      cmd.w = 0.0f;
      cmd.finished = true;
      finished_ = true;
      return cmd;
    }

    // simple point tracking using pure pursuit geometry (lookahead = goal)
    float lx = pts_[0].x;
    float ly = pts_[0].y;
    lookahead_x_ = lx;
    lookahead_y_ = ly;

    // transform lookahead into robot frame
    const float c = std::cos(pose.theta);
    const float s = std::sin(pose.theta);
    const float rx = lx - pose.x;
    const float ry = ly - pose.y;
    const float x_local =  c * rx + s * ry;
    const float y_local = -s * rx + c * ry;

    const float L = hypot2(x_local, y_local);
    const float L2 = (L > 1e-6f) ? (L * L) : 1e-6f;

    float v_des = cfg_.cruise_v;
    if (pts_[0].has_vel) v_des = clamp(pts_[0].v, 0.0f, cfg_.max_v);

    // slowdown near goal
    if (cfg_.slowdown_dist > 0.0f) {
      const float scale = clamp(dist / cfg_.slowdown_dist, 0.0f, 1.0f);
      v_des = clamp(v_des * scale, cfg_.min_v_near_goal, cfg_.max_v);
    }

    // curvature = 2*y/L^2  (pure pursuit)
    const float curvature = (2.0f * y_local) / L2;
    float w_des = v_des * curvature;

    // clamp w
    float w_cmd = clamp(w_des, -cfg_.max_w, cfg_.max_w);
    float v_cmd = v_des;

    // optionally reduce v so we keep the same curvature even when w saturates
    if (cfg_.preserve_curvature_on_w_saturation && (std::fabs(w_des) > cfg_.max_w)) {
      if (std::fabs(curvature) > 1e-6f) {
        v_cmd = clamp(w_cmd / curvature, 0.0f, cfg_.max_v);
      }
    }

    cmd.v = v_cmd;
    cmd.w = w_cmd;
    cmd.lookahead_x = lx;
    cmd.lookahead_y = ly;
    cmd.finished = false;
    finished_ = false;
    return cmd;
  }

  // normal case: multi-point path

  // advance progress along path if we're close enough to the next waypoint
  advanceIfNeeded(pose);

  // if we reached the end segment index and are basically at the final point, finish
  const Waypoint& last = pts_[count_ - 1];
  const float dx_goal = last.x - pose.x;
  const float dy_goal = last.y - pose.y;
  const float dist_goal = hypot2(dx_goal, dy_goal);

  if (dist_goal <= cfg_.pos_tol && seg_i_ >= (count_ - 2)) {
    // optional final heading hold
    if (cfg_.use_final_heading && last.has_heading) {
      const float err_h = wrapAngle(last.heading - pose.theta);
      if (std::fabs(err_h) > cfg_.heading_tol) {
        cmd.v = 0.0f;
        cmd.w = clamp(cfg_.k_heading * err_h, -cfg_.max_w, cfg_.max_w);
        cmd.lookahead_x = last.x;
        cmd.lookahead_y = last.y;
        cmd.finished = false;
        finished_ = false;
        return cmd;
      }
    }

    cmd.v = 0.0f;
    cmd.w = 0.0f;
    cmd.lookahead_x = last.x;
    cmd.lookahead_y = last.y;
    cmd.finished = true;
    finished_ = true;
    return cmd;
  }

  // compute lookahead point on the polyline
  float lx = 0.0f, ly = 0.0f;
  computeLookaheadPoint(pose, &lx, &ly);
  lookahead_x_ = lx;
  lookahead_y_ = ly;

  // transform lookahead into robot frame
  const float c = std::cos(pose.theta);
  const float s = std::sin(pose.theta);
  const float rx = lx - pose.x;
  const float ry = ly - pose.y;
  const float x_local =  c * rx + s * ry;
  const float y_local = -s * rx + c * ry;

  const float L = hypot2(x_local, y_local);
  const float L2 = (L > 1e-6f) ? (L * L) : 1e-6f;

  // choose desired speed:
  // - use waypoint speed if available (we sample from the segment end waypoint)
  // - otherwise use cruise_v
  const Waypoint& seg_end = pts_[seg_i_ + 1];
  float v_des = cfg_.cruise_v;
  if (seg_end.has_vel) v_des = seg_end.v;

  v_des = clamp(v_des, 0.0f, cfg_.max_v);

  // slowdown near the final goal
  if (cfg_.slowdown_dist > 0.0f) {
    const float scale = clamp(dist_goal / cfg_.slowdown_dist, 0.0f, 1.0f);
    v_des = clamp(v_des * scale, cfg_.min_v_near_goal, cfg_.max_v);
  }

  // curvature = 2*y/L^2  (pure pursuit)
  const float curvature = (2.0f * y_local) / L2;
  float w_des = v_des * curvature;

  float w_cmd = clamp(w_des, -cfg_.max_w, cfg_.max_w);
  float v_cmd = v_des;

  // keep the turn radius correct when w saturates (simple but very effective!!!!)
  if (cfg_.preserve_curvature_on_w_saturation && (std::fabs(w_des) > cfg_.max_w)) {
    if (std::fabs(curvature) > 1e-6f) {
      v_cmd = clamp(w_cmd / curvature, 0.0f, cfg_.max_v);
    }
  }

  cmd.v = v_cmd;
  cmd.w = w_cmd;
  cmd.lookahead_x = lx;
  cmd.lookahead_y = ly;
  cmd.finished = false;
  finished_ = false;
  return cmd;
}

void TrajectoryController::advanceIfNeeded(const Pose2D& pose) {
  // if we're already at the last segment, don't advance
  if (seg_i_ >= (count_ - 2)) return;

  // if close enough to the next waypoint, advance
  const Waypoint& next = pts_[seg_i_ + 1];
  const float dx = next.x - pose.x;
  const float dy = next.y - pose.y;
  const float dist = hypot2(dx, dy);

  if (dist <= cfg_.advance_tol) {
    seg_i_++;
    if (seg_i_ > (count_ - 2)) seg_i_ = (count_ - 2);
  }
}

void TrajectoryController::computeLookaheadPoint(const Pose2D& pose, float* lx, float* ly) {
  // default to last point if something goes wrong
  *lx = pts_[count_ - 1].x;
  *ly = pts_[count_ - 1].y;

  // starting segment
  size_t i = seg_i_;
  if (i >= (count_ - 1)) i = count_ - 2;

  // we walk forward along segments by lookahead distance
  float remaining = cfg_.lookahead_dist;

  // current “anchor” is the projection of robot onto the current segment
  float ax = pts_[i].x;
  float ay = pts_[i].y;
  float bx = pts_[i + 1].x;
  float by = pts_[i + 1].y;

  const float sx = bx - ax;
  const float sy = by - ay;
  const float seg_len2 = sx * sx + sy * sy;

  float px = ax;
  float py = ay;

  if (seg_len2 > 1e-12f) {
    // projection t of robot onto segment [0..1]
    const float rx = pose.x - ax;
    const float ry = pose.y - ay;
    float t = (rx * sx + ry * sy) / seg_len2;
    t = clamp(t, 0.0f, 1.0f);

    px = ax + t * sx;
    py = ay + t * sy;

    // remaining distance left on this segment from projection point to segment end
    const float ex = bx - px;
    const float ey = by - py;
    float seg_rem = hypot2(ex, ey);

    // if the lookahead fits within the remainder of this segment, interpolate right here
    if (remaining <= seg_rem && seg_rem > 1e-9f) {
      const float u = remaining / seg_rem;
      *lx = px + u * ex;
      *ly = py + u * ey;
      return;
    }

    // otherwise, consume this segment remainder and move on
    remaining -= seg_rem;
  }

  // now walk full segments forward until we consume remaining distance
  for (; i < (count_ - 2); i++) {
    ax = pts_[i + 1].x;
    ay = pts_[i + 1].y;
    bx = pts_[i + 2].x;
    by = pts_[i + 2].y;

    const float dx = bx - ax;
    const float dy = by - ay;
    const float seg_len = hypot2(dx, dy);

    if (seg_len < 1e-9f) {
      // zero-length segment, skip
      continue;
    }

    if (remaining <= seg_len) {
      const float u = remaining / seg_len;
      *lx = ax + u * dx;
      *ly = ay + u * dy;
      return;
    }

    remaining -= seg_len;
  }

  // if we run out, keep last point
  *lx = pts_[count_ - 1].x;
  *ly = pts_[count_ - 1].y;
}

void TrajectoryController::chassisToWheelSpeeds(float v, float w, float track_width,
                                               float* left, float* right) {
  if (!left || !right) return;

  // track_width is meters between left/right wheel contact patches (or effective width)
  const float half = 0.5f * track_width;
  *left = v - w * half;
  *right = v + w * half;
}

float TrajectoryController::clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

bool TrajectoryController::isFinite(float x) {
  return (x == x) && (x > -3.4e38f) && (x < 3.4e38f);  // filters NaN/Inf
}

float TrajectoryController::wrapAngle(float a) {
  // wrap into [-pi, pi]
  const float kPi = 3.14159265358979323846f;

  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
  return a;
}

float TrajectoryController::hypot2(float x, float y) {
  return std::sqrt(x * x + y * y);
}

float TrajectoryController::sanitizePositive(float x, float fallback) {
  if (!isFinite(x)) return fallback;
  if (x < 0.0f) return -x;
  return x;
}
