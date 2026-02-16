#ifndef SECBOT_NAVIGATION_PATH_SMOOTHER_HPP
#define SECBOT_NAVIGATION_PATH_SMOOTHER_HPP
#include <utility>
#include <vector>

#include "secbot_navigation/grid_map.hpp"

namespace secbot_navigation {

struct PathSmootherConfig {
  int max_skip = 1;
};

// === Post-processes Grid Paths to make them smoother ===
class PathSmoother {
 public:
  // === Constructor ===
  explicit PathSmoother(const GridMap& grid_map,
                        PathSmootherConfig config = PathSmootherConfig());

  // === Smooths a given path ===
  using Cell = GridMap::Cell;
  std::vector<Cell> smooth(const std::vector<Cell>& path);

 private:
  bool _line_is_clear(const Cell& p1, const Cell& p2);

  const GridMap& grid_map_;
  PathSmootherConfig config_;
  size_t max_skip_;
};

}  // namespace secbot_navigation

#endif
