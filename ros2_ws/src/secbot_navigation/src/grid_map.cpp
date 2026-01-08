#include "secbot_navigation/grid_map.hpp"

namespace secbot_navigation
{

  // === Constructor ===
  GridMap::GridMap(const std::vector<std::vector<int>> &grid) : grid_(grid)
  {
    rows_ = grid.size();
    cols_ = (rows_ > 0) ? grid[0].size() : 0;
  }

  // === Getters ===
  int GridMap::get_rows() const { return rows_; }

  int GridMap::get_cols() const { return cols_; }

  // === Checking Bounds ===
  bool GridMap::in_bounds(const std::pair<int, int> &cell) const
  {
    int r = cell.first;
    int c = cell.second;
    return r >= 0 && r < rows_ && c >= 0 && c < cols_;
  }

  // === Check Traversability ===
  bool GridMap::is_free(const std::pair<int, int> &cell) const
  {
    int r = cell.first;
    int c = cell.second;
    return grid_[r][c] == 0;
  }

  // === Get Neighbors ===
  std::vector<std::pair<int, int>>
  GridMap::neighbors(const std::pair<int, int> &cell) const
  {
    std::vector<std::pair<int, int>> result;
    int r = cell.first;
    int c = cell.second;

    // Directions: Up, Down, Left, Right
    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    for (int i = 0; i < 4; ++i)
    {
      std::pair<int, int> n = {r + dr[i], c + dc[i]};
      if (in_bounds(n) && is_free(n))
      {
        result.push_back(n);
      }
    }
    return result;
  }

  // === Set Obstacle ===
  void GridMap::set_obstacle(const std::pair<int, int> &cell)
  {
    if (in_bounds(cell))
    {
      grid_[cell.first][cell.second] = 1;
    }
  }

}
