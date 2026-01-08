#ifndef SECBOT_NAVIGATION_TRAJECTORY_HPP
#define SECBOT_NAVIGATION_TRAJECTORY_HPP

#include <tuple>
#include <vector>

namespace secbot_navigation
{

  struct TrajectoryPoint
  {
    double x;
    double y;
    double theta;
    double v;

    TrajectoryPoint(double x_val, double y_val, double theta_val = 0.0,
                    double v_val = 0.0)
        : x(x_val), y(y_val), theta(theta_val), v(v_val) {}
  };

  struct TrajectoryConfig
  {
    double speed = 0.5;
  };

  class Trajectory
  {
  public:
    Trajectory(TrajectoryConfig config = TrajectoryConfig());

    void add_point(double x, double y, double v = 0.5);
    void compute_headings();

    std::vector<TrajectoryPoint>::const_iterator begin() const;
    std::vector<TrajectoryPoint>::const_iterator end() const;
    const std::vector<TrajectoryPoint> &get_points() const;

  private:
    std::vector<TrajectoryPoint> points_;
  };

  // === PurePursuitController definition ===
  class PurePursuitController
  {
  public:
    PurePursuitController(double track_width = 0.5, double lookahead_dist = 1.0,
                          double max_v = 1.0, double max_w = 2.0,
                          double slow_dist = 2.0);

    void set_path(const Trajectory &path);

    std::pair<double, double>
    compute_curvature_and_velocity(std::pair<double, double> local_point,
                                   double current_v);

    TrajectoryPoint find_lookahead_point(double robot_x, double robot_y);

    std::pair<double, double> vector_projection(std::pair<double, double> A,
                                                std::pair<double, double> B,
                                                std::pair<double, double> P);

    double linear_slowdown(double desired_v, double dist_to_goal);

    std::pair<double, double>
    coordinate_transform(std::tuple<double, double, double> robot_pose,
                         std::pair<double, double> target_point);

    std::pair<double, double> differential_drive_kinematics(double v, double w);

    std::pair<double, double>
    compute_control(std::tuple<double, double, double> robot_pose,
                    std::pair<double, double> lookahead_point,
                    std::pair<double, double> goal_point);

    double get_track_width() const { return track_width_; }
    double get_lookahead_dist() const { return lookahead_dist_; }

  private:
    double track_width_;
    double lookahead_dist_;
    double max_v_;
    double max_w_;
    double slow_dist_;
    std::vector<TrajectoryPoint> current_path_points_;
  };

}

#endif 
