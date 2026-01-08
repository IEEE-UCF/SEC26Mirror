#include "secbot_navigation/dstar_lite.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace secbot_navigation
{

  static const double INF = std::numeric_limits<double>::infinity();

  // === Constructor ===
  DStarLite::DStarLite(GridMap &grid_map, Node start, Node goal,
                       DStarLiteConfig config)
      : map_(grid_map), start_(start), goal_(goal), km_(0.0)
  {
    (void)config; 

    int rows = map_.get_rows();
    int cols = map_.get_cols();

    // Initialize grids with Infinite cost
    g_.assign(rows, std::vector<double>(cols, INF));
    rhs_.assign(rows, std::vector<double>(cols, INF));
    // Initialize Open Set tracker
    open_hash_.assign(rows, std::vector<CellInfo>(cols, {false, {0.0, 0.0}}));

    // Initialize Goal Node:
    // In D* Lite. search from Goal to Start.
    rhs_[goal_.first][goal_.second] = 0.0;

    // Add Goal to priority queue
    insert(goal_, calculate_key(goal_));
  }


  // === Heuristic ===
  double DStarLite::manhattan(const Node &a, const Node &b)
  {
    return std::abs(a.first - b.first) + std::abs(a.second - b.second);
  }

  // === Calculate Key ===
  // k1 = min(g, rhs) + h + km (Total estimated cost)
  // k2 = min(g, rhs)          (Cost so far)
  DStarLite::Key DStarLite::calculate_key(const Node &s)
  {
    double min_val = std::min(g_[s.first][s.second], rhs_[s.first][s.second]);
    return {min_val + manhattan(start_, s) + km_, min_val};
  }

  // === Edge Cost ===
  double DStarLite::cost(const Node &a, const Node &b)
  {
    if (!map_.is_free(a) || !map_.is_free(b))
    {
      return INF;
    }
    return 1.0;
  }

  // === Priority Queue Operations ===
  void DStarLite::insert(Node u, Key k)
  {
    U_.insert({k, u});
    CellInfo &info = open_hash_[u.first][u.second];
    info.in_queue = true;
    info.key = k;
  }

  void DStarLite::remove(Node u)
  {
    CellInfo &info = open_hash_[u.first][u.second];
    if (info.in_queue)
    {
      U_.erase({info.key, u});
      info.in_queue = false;
    }
  }

  // === Check consistency between g(u) and rhs(u) ===
  void DStarLite::update_vertex(Node u)
  {
    if (u != goal_)
    {
      double min_rhs = INF;
      // Look at all neighbors to find the best parent
      for (const auto &s : map_.neighbors(u))
      {
        double c = cost(u, s);
        double g_val = g_[s.first][s.second];
        if (c != INF && g_val != INF)
        {
          min_rhs = std::min(min_rhs, c + g_val);
        }
      }
      rhs_[u.first][u.second] = min_rhs;
    }

    remove(u);

    // If node is inconsistent (g != rhs), add to queue for processing
    if (g_[u.first][u.second] != rhs_[u.first][u.second])
    {
      insert(u, calculate_key(u));
    }
  }

  // === Update Start Position ===
  void DStarLite::update_start(Node start)
  {
    if (start != start_)
    {
      // Increment km (key modifier) to account for robot movement
      // This avoids recomputing keys for the entire graph.
      km_ += manhattan(start_, start);
      start_ = start;
    }
  }

  // === Main Compute Loop ===
  void DStarLite::compute_shortest_path()
  {
    while (!U_.empty())
    {
      auto top_it = U_.begin();
      Key k_old = top_it->first;
      Node u = top_it->second;

      Key k_start = calculate_key(start_);

      // Stop Condition:
      // If the smallest key in queue is >= start's key
      // AND start node is consistent (g == rhs), we are done.
      if (k_old >= k_start &&
          rhs_[start_.first][start_.second] == g_[start_.first][start_.second])
      {
        break;
      }

      U_.erase(top_it);
      open_hash_[u.first][u.second].in_queue = false;

      Key k_new = calculate_key(u);

      // Case 1: Node key is outdated (lazy update)
      if (k_old < k_new)
      {
        insert(u, k_new);
      }
      // Case 2: Overconsistent (g > rhs) - classic Dijkstra/A* relaxation
      else if (g_[u.first][u.second] > rhs_[u.first][u.second])
      {
        g_[u.first][u.second] = rhs_[u.first][u.second];
        for (const auto &s : map_.neighbors(u))
        {
          update_vertex(s);
        }
      }
      // Case 3: Underconsistent (g < rhs) - occurs when obstacle appears
      else
      {
        g_[u.first][u.second] = INF;
        update_vertex(u);
        for (const auto &s : map_.neighbors(u))
        {
          update_vertex(s);
        }
      }
    }
  }

  // === Extract Path ===
  std::vector<DStarLite::Node> DStarLite::extract_path()
  {
    if (rhs_[start_.first][start_.second] == INF)
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
        double g_val = g_[s.first][s.second];
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
