#include "secbot_navigation/grid_map.hpp"

namespace secbot_navigation {

// === Constructor ===
GridMap::GridMap(const std::vector<std::vector<int>> &grid) : grid_(grid) {
  rows_ = grid.size();                       // size of map horizontally
  cols_ = (rows_ > 0) ? grid[0].size() : 0;  // size of map vertically
}

int GridMap::get_rows() const { return rows_; }  // get the rows

int GridMap::get_cols() const { return cols_; }  // get the columns

// === Checking Bounds ===
bool GridMap::in_bounds(const Cell &cell) const {
  return cell.r >= 0 && cell.r < rows_ && cell.c >= 0 && cell.c < cols_;
}

// === Check Traversability ===
bool GridMap::is_free(const Cell &cell) const {
  return grid_[cell.r][cell.c] == 0;
}

// === Get Neighbors ===
std::vector<GridMap::Cell> GridMap::neighbors(const Cell &cell) const {
  std::vector<Cell> result;
  int r = cell.r;
  int c = cell.c;

  // Directions: Up, Down, Left, Right
  int dr[] = {-1, 1, 0, 0};
  int dc[] = {0, 0, -1, 1};

  for (int i = 0; i < 4; ++i) {
    Cell n = {r + dr[i], c + dc[i]};
    if (in_bounds(n) && is_free(n)) {
      result.push_back({n.r, n.c});
    }
  }
  return result;
}

// === Set Obstacle ===
void GridMap::set_obstacle(const Cell &cell) {
  if (in_bounds(cell)) {
    grid_[cell.r][cell.c] = 1;
  }
}

}  // namespace secbot_navigation
