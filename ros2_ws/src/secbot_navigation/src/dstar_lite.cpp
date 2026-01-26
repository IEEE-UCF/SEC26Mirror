#include "secbot_navigation/dstar_lite.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace secbot_navigation
{

  static const double INF = std::numeric_limits<double>::infinity();

  // === Constructor ===
  DStarLite::DStarLite(GridMap &grid_map, Node start, Node goal, DStarLiteConfig config): map_(grid_map), start_(start), goal_(goal), offset_old_poriority_(0.0)
  {
    (void)config; 

    int rows = map_.get_rows();
    int cols = map_.get_cols();

    // Initialize grids with Infinite cost
    // === USED IN HEURISTICS ===
    g_.assign(rows, std::vector<double>(cols, INF)); // this is the cost from the node to the goal
    rhs_.assign(rows, std::vector<double>(cols, INF)); // this is the lookahead cost (rhs(s) = min(cost(s, s') + g(s')))
    // Initialize Open Set tracker
    open_hash_.assign(rows, std::vector<CellInfo>(cols, {false, {0.0, 0.0}}));

    // Initialize Goal Node:
    // In D* Lite. search from Goal to Start.
    rhs_[goal_.r][goal_.c] = 0.0;

    // Add Goal to priority queue
    insert(goal_, calculate_priority(goal_));
  }


  // === Heuristic ===
  // The manhattan distance h(a,b) = |r_a - r_b| + |c_a - c_b|
  double DStarLite::distance_between_points(const Node &a, const Node &b) const
  {
    return std::abs(a.r - b.r) + std::abs(a.c - b.c);
  }

  // === Calculate Key ===
  // k1 = min(g, rhs) + h + offset_old_poriority (Total estimated cost)
  // k2 = min(g, rhs)          (Cost so far)
  DStarLite::Priority DStarLite::calculate_priority(const Node &s) const
  {

    // === this is the real heuristics ===
    // min(g, rhs) known aass cost to go from s -> goal (biases search towards nodes closer to the robot)
    double min_val = std::min(g_[s.r][s.c], rhs_[s.r][s.c]);

    // lower k1 higher priority           k2 prevents oscillation (stability)
    return {min_val + distance_between_points(start_, s) + offset_old_poriority_, min_val}; 
  }

  // === Edge Cost ===
  double DStarLite::cost(const Node &a, const Node &b) const
  {
    if (!map_.is_free(a) || !map_.is_free(b))
    {
      return INF;
    }
    return 1.0;
  }

  // === Priority Queue Operations ===
  void DStarLite::insert(Node n, Priority p)
  {
    nodes_to_process_.insert({p, n});
    CellInfo &info = open_hash_[n.r][n.c];
    info.in_queue = true;
    info.prio = p;
  }

  void DStarLite::remove(Node n)
  {
    CellInfo &info = open_hash_[n.r][n.c];
    if (info.in_queue)
    {
      nodes_to_process_.erase({info.prio, n});
      info.in_queue = false;
    }
  }

  // === Check consistency between g(n) and rhs(n) ===
  void DStarLite::update_vertex(Node n)
  {
    if (n != goal_)
    {
      double min_rhs = INF;
      // Look at all neighbors to find the best parent
      for (const auto &s : map_.neighbors(n))
      {
        double c = cost(n, s);
        double g_val = g_[s.r][s.c];
        if (c != INF && g_val != INF)
        {
          min_rhs = std::min(min_rhs, c + g_val);
        }
      }
      rhs_[n.r][n.c] = min_rhs;
    }

    remove(n);

    // If node is inconsistent (g != rhs), add to queue for processing
    if (g_[n.r][n.c] != rhs_[n.r][n.c])
    {
      insert(n, calculate_priority(n));
    }
  }

  // === Update Start Position ===
  void DStarLite::update_start(Node start)
  {
    if (start != start_)
    {
      // Increment offset_old_poriority (priority modifier) to account for robot movement
      // This avoids recomputing prioritys for the entire graph.
      offset_old_poriority_ += distance_between_points(start_, start);  // this is HUGE ( when the robot moves the distance changes - 
                                                                        // instead of recomputing D* Lite shifts priority to offset_old_poriority_ also known as km)
      start_ = start;
    }
  }

  // === Main Compute Loop ===
  void DStarLite::compute_shortest_path()
  {
    while (!nodes_to_process_.empty())
    {
      auto top_it = nodes_to_process_.begin();
      Priority p_old = top_it->first;
      Node n = top_it->second;

      Priority p_start = calculate_priority(start_);

      // Stop Condition:
      // If the smallest priority in queue is >= start's priority
      // AND start node is consistent (g == rhs), we are done.
      if (p_old >= p_start &&
          rhs_[start_.r][start_.c] == g_[start_.r][start_.c])
      {
        break;
      }

      nodes_to_process_.erase(top_it);
      open_hash_[n.r][n.c].in_queue = false;

      Priority p_new = calculate_priority(n);

      // Case 1: Node key is outdated (lazy update)
      if (p_old < p_new)
      {
        insert(n, p_new);
      }
      // Case 2: Overconsistent (g > rhs) - classic Dijkstra/A* relaxation
      else if (g_[n.r][n.c] > rhs_[n.r][n.c])
      {
        g_[n.r][n.c] = rhs_[n.r][n.c];
        for (const auto &s : map_.neighbors(n))
        {
          update_vertex(s);
        }
      }
      // Case 3: Underconsistent (g < rhs) - occurs when obstacle appears
      else
      {
        g_[n.r][n.c] = INF;
        update_vertex(n);
        for (const auto &s : map_.neighbors(n))
        {
          update_vertex(s);
        }
      }
    }
  }

  // === Extract Path ===
  std::vector<DStarLite::Node> DStarLite::extract_path()
  {
    if (rhs_[start_.r][start_.c] == INF)
    {
      return {};
    }

    std::vector<Node> path;
    path.push_back(start_);
    Node current = start_;

    int max_steps = map_.get_rows() * map_.get_cols();
    int steps = 0;

    while (current != goal_ && steps++ < max_steps)
    {
      Node best_node = current;
      double min_cost = INF;

      // Gradient descent: pick neighbor with lowest g-value + traversal cost
      for (const auto &s : map_.neighbors(current))
      {
        double c = cost(current, s);
        double g_val = g_[s.r][s.c];
        if (c != INF && g_val != INF)
        {
          if (c + g_val < min_cost)
          {
            min_cost = c + g_val;
            best_node = s;
          }
        }
      }

      if (best_node == current || min_cost == INF)
      {
        return {};
      }

      current = best_node;
      path.push_back(current);
    }

    return path;
  }

}
