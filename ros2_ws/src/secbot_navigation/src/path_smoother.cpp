#include "secbot_navigation/path_smoother.hpp"
#include <algorithm>

namespace secbot_navigation
{

  PathSmoother::PathSmoother(PathSmootherConfig config) : config_(config)
  {
    max_skip_ = config_.iterations;
  }

  // === Smoothing Algorithm ===
  // Logic:
  // 1. Start at index i 
  // 2. Look ahead up to 'max_skip' points to index j.
  // 3. Backward check from j to i: can we connect i -> j directly?
  // 4. If yes, add j to smoothed path and advance i to j.
  // 5. If no, reduce j and try again.
  std::vector<std::pair<int, int>>
  PathSmoother::smooth(const std::vector<std::pair<int, int>> &path)
  {
    if (path.size() < 3)
    {
      return path;
    }

    std::vector<std::pair<int, int>> smoothed;
    smoothed.push_back(path[0]); // Always keep start

    size_t i = 0;
    while (i < path.size() - 1)
    {
      // Look ahead window
      size_t j = std::min(i + max_skip_, path.size() - 1);

      // Try to find furthest reachable point in window
      while (j > i + 1)
      {
        if (_line_is_clear(path[i], path[j]))
        {
          break; // Found shortcut
        }
        j--;
      }

      // Add the point we found
      smoothed.push_back(path[j]);
      i = j;
    }

    return smoothed;
  }

  // === Line of Sight Check ===
  bool PathSmoother::_line_is_clear(const std::pair<int, int> &p1,
                                    const std::pair<int, int> &p2)
  {
    // TODO: Implement Raycasting (Bresenham's Line Algorithm)
    // For now, checks are disabled (assumes clear), preserving Python behavior.
    // In a real robot, checking the grid_map along the line is required.
    return true;
  }

}
