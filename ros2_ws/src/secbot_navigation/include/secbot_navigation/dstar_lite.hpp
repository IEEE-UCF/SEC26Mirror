#ifndef SECBOT_NAVIGATION_DSTAR_LITE_HPP
#define SECBOT_NAVIGATION_DSTAR_LITE_HPP

#include "secbot_navigation/grid_map.hpp"
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace secbot_navigation
{

  struct DStarLiteConfig
  {
    int max_steps = 50000;
  };

  // === Implements the D* Lite algorithm ===
  class DStarLite
  {
  public:
    using Key = std::pair<double, double>;
    using Node = std::pair<int, int>;

    // === Initializes the DStarLite planner ===
    DStarLite(GridMap &grid_map, Node start, Node goal,
              DStarLiteConfig config = DStarLiteConfig());

    void update_start(Node start);
    void compute_shortest_path();
    std::vector<Node> extract_path();

  private:
    double manhattan(const Node &a, const Node &b);
    Key calculate_key(const Node &s);
    double cost(const Node &a, const Node &b);

    void insert(Node u, Key k);
    void remove(Node u);
    void update_vertex(Node u);

    GridMap &map_;
    Node start_;
    Node goal_;
    double km_;

    std::vector<std::vector<double>> g_;
    std::vector<std::vector<double>> rhs_;

    struct CellInfo
    {
      bool in_queue = false;
      Key key;
    };
    std::vector<std::vector<CellInfo>> open_hash_;

    std::set<std::pair<Key, Node>> U_;
  };

} 

#endif