#ifndef SECBOT_NAVIGATION_PATH_SMOOTHER_HPP
#define SECBOT_NAVIGATION_PATH_SMOOTHER_HPP

#include <utility>
#include <vector>

namespace secbot_navigation
{

  struct PathSmootherConfig
  {
    int iterations = 50;
    double alpha = 0.5;
    double beta = 0.2;
    double max_step = 0.05;
  };

  // === Post-processes Grid Paths to make them smoother ===
  class PathSmoother
  {
  public:
    // === Constructor ===
    explicit PathSmoother(PathSmootherConfig config = PathSmootherConfig());

    // === Smooths a given path ===
    std::vector<std::pair<int, int>>
    smooth(const std::vector<std::pair<int, int>> &path);

  private:
    bool _line_is_clear(const std::pair<int, int> &p1,
                        const std::pair<int, int> &p2);

    PathSmootherConfig config_;
    int max_skip_; 
  };

} 

#endif 
