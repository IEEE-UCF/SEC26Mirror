#ifndef SECBOT_NAVIGATION_GRID_MAP_HPP
#define SECBOT_NAVIGATION_GRID_MAP_HPP

#include <utility>
#include <vector>

namespace secbot_navigation
{

  // === Represents the 2D environment for the robot ===

  // Used by:
  //   - DStarLite: To know which cells are traversable.
  //   - PlannerServer: To initialize the planning environment.
  //   - PathingNode: To load the initial map from arena_layout.yaml.
  
  class GridMap
  {
  public:
    // === Constructor ===
    GridMap(const std::vector<std::vector<int>> &grid);

    int get_rows() const;
    int get_cols() const;

    // === Checks if a cell is within the grid boundaries ===
    bool in_bounds(const std::pair<int, int> &cell) const;

    // === Checks if a cell is traversable ===
    bool is_free(const std::pair<int, int> &cell) const;

    // === Marks a cell as an obstacle ===
    void set_obstacle(const std::pair<int, int> &cell);

    // === Gets valid neighboring cells ===
    std::vector<std::pair<int, int>>
    neighbors(const std::pair<int, int> &cell) const;

  private:
    std::vector<std::vector<int>> grid_;
    int rows_;
    int cols_;
  };

}

#endif
