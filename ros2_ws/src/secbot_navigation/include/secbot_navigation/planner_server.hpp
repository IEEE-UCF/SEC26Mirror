#ifndef SECBOT_NAVIGATION_PLANNER_SERVER_HPP
#define SECBOT_NAVIGATION_PLANNER_SERVER_HPP

#include "secbot_navigation/dstar_lite.hpp"
#include "secbot_navigation/grid_map.hpp"
#include "secbot_navigation/path_smoother.hpp"
#include "secbot_navigation/trajectory.hpp"
#include <memory>
#include <optional>
#include <stdexcept>

namespace secbot_navigation
{

  struct PlannerConfig
  {
    int max_skip = 1;   // Smoothing parameter (how many points to skip)
    double speed = 0.5; // Default robot speed in m/s
  };

  
  // Contains:
  // 1. The GridMap (Environment)
  // 2. The DStarLite Planner (Algorithm)
  // 3. The Path Smoother (Post-processing)
  class PlannerServer
  {
  public:
    // === Initializes the server with map and config ===
    PlannerServer(GridMap &grid_map, const PlannerConfig &config)
        : grid_map_(grid_map), config_(config)
    {
      PathSmootherConfig ps_config;
      ps_config.iterations = config.max_skip;
      smoother_ = std::make_unique<PathSmoother>(ps_config);
    }

    // === Sets or updates the goal position ===
    void set_goal(const std::pair<int, int> &goal)
    {
      goal_ = goal;
      planner_.reset();
    }

    // ===Main planning function===
    std::optional<Trajectory> compute_plan(const std::pair<int, int> &start)
    {
      if (!goal_)
      {
        throw std::runtime_error("Goal not set");
      }

      if (!planner_)
      {
        // 1. Check if goal is set.
        planner_ = std::make_unique<DStarLite>(grid_map_, start, *goal_);
      }
      else
      {
        // 2. Initialize or Update D* Lite planner.
        planner_->update_start(start);
      }
      // 3. Compute shortest path (Grid Indices).
      planner_->compute_shortest_path();
      auto raw_path = planner_->extract_path();

      if (raw_path.empty())
      {
        return std::nullopt;
      }

      // 4. Smooth the path to remove unnecessary waypoints.
      auto smooth_path = smoother_->smooth(raw_path);

      // 5. Convert to World Trajectory (Meters & Headings).
      return _build_trajectory(smooth_path);
    }

    // === Safety check ===
    bool trajectory_blocked(const Trajectory &trajectory)
    {
      for (const auto &p : trajectory)
      {
        if (!grid_map_.is_free({static_cast<int>(p.x), static_cast<int>(p.y)}))
        {
          return true;
        }
      }
      return false;
    }

  private:
    // === Converts a list of grid cells to a trajectory with physics properties ===
    // Assigns speed and computes orientation (theta) for each point.
    Trajectory _build_trajectory(const std::vector<std::pair<int, int>> &path)
    {
      Trajectory traj;
      for (const auto &p : path)
      {
        traj.add_point(static_cast<double>(p.first),
                       static_cast<double>(p.second), config_.speed);
      }
      traj.compute_headings(); // Calculate theta for each point
      return traj;
    }

    GridMap &grid_map_;
    PlannerConfig config_;

    std::unique_ptr<PathSmoother> smoother_;
    std::unique_ptr<DStarLite> planner_;
    std::optional<std::pair<int, int>> goal_;
  };

}

#endif
