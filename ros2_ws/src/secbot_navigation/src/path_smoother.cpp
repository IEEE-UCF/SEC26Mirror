#include "secbot_navigation/path_smoother.hpp"

#include <algorithm>
#include <cstdlib>

#include "secbot_navigation/grid_map.hpp"

namespace secbot_navigation {

PathSmoother::PathSmoother(const GridMap& grid_map, PathSmootherConfig config)
    : grid_map_(grid_map),
      config_(config),
      max_skip_(static_cast<size_t>(config.max_skip)) {}

// === Smoothing Algorithm ===
// Logic:
// 1. Start at index i
// 2. Look ahead up to 'max_skip' points to index j.
// 3. Backward check from j to i: can we connect i -> j directly?
// 4. If yes, add j to smoothed path and advance i to j.
// 5. If no, reduce j and try again.
std::vector<PathSmoother::Cell> PathSmoother::smooth(
    const std::vector<Cell>& path) {
  if (path.size() < 3) {
    return path;
  }

  std::vector<Cell> smoothed;
  smoothed.push_back(path[0]);  // Always keep start

  size_t i = 0;
  while (i < path.size() - 1) {
    // Look ahead window
    size_t j = std::min(i + max_skip_, path.size() - 1);

    // Try to find furthest reachable point in window
    while (j > i + 1) {
      if (_line_is_clear(path[i], path[j])) {
        break;  // Found shortcut
      }
      j--;
    }

    // Add the point we found
    smoothed.push_back(path[j]);
    i = j;
  }

  return smoothed;
}

// === Bresenham's Line Algorithm ===
bool PathSmoother::_line_is_clear(const Cell& p1, const Cell& p2) {
  int x0 = p1.r;
  int y0 = p1.c;
  int x1 = p2.r;
  int y1 = p2.c;

  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;

  int err = dx - dy;

  while (true) {
    GridMap::Cell cell{x0, y0};

    if (!grid_map_.in_bounds(cell) || !grid_map_.is_free(cell)) return false;

    if (x0 == x1 && y0 == y1) break;

    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  return true;
}

}  // namespace secbot_navigation
