#ifndef SECBOT_NAVIGATION_GRID_MAP_HPP
#define SECBOT_NAVIGATION_GRID_MAP_HPP

#include <utility>
#include <vector>

namespace secbot_navigation
{

  // === Represents the 2D environment for the robot ===

  // Used by:
  //   - DStarLite: To know which cells are traversable.
  //   - PathSmoother: To check if line is clear to proceed on using Bresenham's algorithm
  //   - PlannerServer: To initialize the planning environment.
  //   - PathingNode: To load the initial map from arena_layout.yaml.
  
  class GridMap
  {
  public:

    struct Cell { 
                  int r; int c; 
                  bool operator==(const Cell& cell) const { return r == cell.r && c == cell.c; }
                  bool operator!=(const Cell& cell) const { return !(*this == cell); }
                  bool operator<(const Cell& cell) const 
                  {
                    return (r < cell.r) || (r == cell.r && c < cell.c);
                  }
                };
    // === Constructor ===
    GridMap(const std::vector<std::vector<int>> &grid);

    int get_rows() const; // gets the rows
    int get_cols() const; // gets the columns

    // === Checks if a cell is within the grid boundaries ===
    bool in_bounds(const Cell &cell) const;  

    // === Checks if a cell is traversable ===
    bool is_free(const Cell &cell) const;

    // === Marks a cell as an obstacle ===
    void set_obstacle(const Cell &cell);

    // === Gets valid neighboring cells ===
    std::vector<GridMap::Cell> neighbors(const Cell &cell) const;

  private:
    std::vector<std::vector<int>> grid_;
    int rows_;
    int cols_;
  };

}

#endif
