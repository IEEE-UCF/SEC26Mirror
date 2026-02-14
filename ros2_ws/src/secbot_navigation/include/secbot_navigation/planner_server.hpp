#ifndef SECBOT_NAVIGATION_PLANNER_SERVER_HPP
#define SECBOT_NAVIGATION_PLANNER_SERVER_HPP

#include <memory>
#include <optional>
#include <stdexcept>

#include "secbot_navigation/dstar_lite.hpp"
#include "secbot_navigation/grid_map.hpp"
#include "secbot_navigation/path_smoother.hpp"
#include "secbot_navigation/trajectory.hpp"

namespace secbot_navigation {

struct PlannerConfig {
  int max_skip = 1;    // max skipping if no skips provided
  double speed = 0.5;  // robot speed if no speed provided
};

class PlannerServer {
 public:
  using Cell = GridMap::Cell;

  // === Initializes the server with map and config ===
  PlannerServer(GridMap &grid_map, const PlannerConfig &config)
      : grid_map_(grid_map), config_(config) {
    PathSmootherConfig ps_config;
    ps_config.max_skip = config.max_skip;
    smoother_ = std::make_unique<PathSmoother>(grid_map_, ps_config);
  }

  // === Sets or updates the goal position ===
  void set_goal(const Cell &goal) {
    goal_ = goal;
    planner_.reset();
  }

  // ===Main planning function===
  std::optional<Trajectory> compute_plan(const Cell &start) {
    if (!goal_) {
      throw std::runtime_error("Goal not set");
    }

    // Checks if there is a plan set
    if (!planner_)  // If not generate one
    {
      planner_ = std::make_unique<DStarLite>(grid_map_, start, *goal_);
    } else  // If there is one update the start
    {
      planner_->update_start(start);
    }

    // Compute the shortest path
    planner_->compute_shortest_path();
    auto raw_path = planner_->extract_path();

    if (raw_path.empty())  // No path generated
    {
      return std::nullopt;
    }

    auto smooth_path = smoother_->smooth(
        raw_path);  // Smooth the path removing unneccesary points

    return _build_trajectory(
        smooth_path);  // Use trajectory class and convert points for robot from
                       // global -> local
  }

 private:
  // === Converts a list of grid cells to a trajectory with physics properties
  // === Assigns speed and computes orientation (theta) for each point.
  Trajectory _build_trajectory(const std::vector<Cell> &path)  // Build the path
  {
    Trajectory traj;
    for (const auto &p : path) {
      traj.add_point(static_cast<double>(p.r), static_cast<double>(p.c),
                     config_.speed);
    }
    traj.compute_direction();  // Calculate theta for each point
    return traj;
  }

  // configs
  GridMap &grid_map_;
  PlannerConfig config_;

  std::unique_ptr<PathSmoother> smoother_;
  std::unique_ptr<DStarLite> planner_;
  std::optional<Cell> goal_;
};

}  // namespace secbot_navigation

#endif
