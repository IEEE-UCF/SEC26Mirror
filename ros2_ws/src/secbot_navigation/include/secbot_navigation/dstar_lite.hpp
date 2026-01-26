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
    int max_steps = 0; // just in case grid fails to create rows and columns
  };

  // === Implements the D* Lite algorithm ===
  class DStarLite
  {
  public:
    using Priority = std::pair<double, double>; // finds the priority of g to rhs
    using Node = GridMap::Cell;

    // === Initializes the DStarLite planner ===
    DStarLite(GridMap &grid_map, Node start, Node goal, DStarLiteConfig config = DStarLiteConfig());

    void update_start(Node start);  // update the start
    void compute_shortest_path();   // computes the shortest path, used in re planning
    std::vector<Node> extract_path(); // show the path

  private:
    double distance_between_points(const Node &a, const Node &b) const; // find distance between 2 nodes
    Priority calculate_priority(const Node &s) const; // find the cost so far
    double cost(const Node &a, const Node &b) const; // finds if node is free on the map

    void insert(Node n, Priority p);
    void remove(Node n);
    void update_vertex(Node n);


    // configs
    GridMap &map_;
    Node start_;
    Node goal_;
    double offset_old_poriority_; // also known as km which is just lazily given the old priority (HEURISTICS)

    std::vector<std::vector<double>> g_;
    std::vector<std::vector<double>> rhs_;

    struct CellInfo
    {
      bool in_queue = false;
      Priority prio;
    };
    std::vector<std::vector<CellInfo>> open_hash_;

    std::set<std::pair<Priority, Node>> nodes_to_process_;
  };

} 

#endif