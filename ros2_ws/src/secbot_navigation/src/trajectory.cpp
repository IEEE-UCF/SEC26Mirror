#include "secbot_navigation/trajectory.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace secbot_navigation
{

    // === Constructor that takes a config ===
    Trajectory::Trajectory(TrajectoryConfig config) { (void)config; }


    // === Adds a point to the path list ===
    void Trajectory::add_point(double x, double y, double v)
    {
        points_.emplace_back(x, y, 0.0, v);
    }

    // === Calculates the eheading angle that points towards the next point ===
    void Trajectory::compute_headings()
    {
        for (size_t i = 0; i < points_.size() - 1; ++i)
        {
            double dx = points_[i + 1].x - points_[i].x;
            double dy = points_[i + 1].y - points_[i].y;
            points_[i].theta = std::atan2(dy, dx);
        }
    }

    // === Helpers ===
    std::vector<TrajectoryPoint>::const_iterator Trajectory::begin() const
    {
        return points_.begin();
    }

    std::vector<TrajectoryPoint>::const_iterator Trajectory::end() const
    {
        return points_.end();
    }

    const std::vector<TrajectoryPoint> &Trajectory::get_points() const
    {
        return points_;
    }

    // === Pure Pursuit Implementation ===
    // Controls outputs of left/right wheel velocities

    // Constructor stores params (track_width, lookahead_dist, max_v, max_w, slow_dist)
    PurePursuitController::PurePursuitController(double track_width,
                                                 double lookahead_dist,
                                                 double max_v, double max_w,
                                                 double slow_dist)
        : track_width_(track_width), lookahead_dist_(lookahead_dist), max_v_(max_v),
          max_w_(max_w), slow_dist_(slow_dist) {}


    // === Sets/updates current path controller should follow ===
    void PurePursuitController::set_path(const Trajectory &path)
    {
        // Store the path locally
        current_path_points_.clear();
        for (const auto &p : path.get_points())
        {
            current_path_points_.push_back(p);
        }
    }


    // === Projects a point P onto segment AB ===
    std::pair<double, double>
    PurePursuitController::vector_projection(std::pair<double, double> A,
                                             std::pair<double, double> B,
                                             std::pair<double, double> P)
    {

        double ax = A.first;
        double ay = A.second;
        double bx = B.first;
        double by = B.second;
        double px = P.first;
        double py = P.second;

        double ab_x = bx - ax;
        double ab_y = by - ay;
        double ap_x = px - ax;
        double ap_y = py - ay;

        // dot products for projection to scalar t
        double dot_ap_ab = ap_x * ab_x + ap_y * ab_y;
        double dot_ab_ab = ab_x * ab_x + ab_y * ab_y;

        if (dot_ab_ab == 0.0)
        {
            return A;
        }

        double t = dot_ap_ab / dot_ab_ab;

        // Keep between [0,1]
        t = std::max(0.0, std::min(1.0, t));


        // A + t * (B-A)
        return {ax + t * ab_x, ay + t * ab_y};
    }


    // === Find llookahead point "carrot" on path ===
    TrajectoryPoint PurePursuitController::find_lookahead_point(double robot_x,
                                                                double robot_y)
    {
        if (current_path_points_.empty())
        {
            return {0, 0, 0, 0};
        }

        // 1. Find closest point on path segments (Projected)
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_idx = 0;
        std::pair<double, double> best_proj = {robot_x, robot_y};

        // Iterate segments
        for (size_t i = 0; i < current_path_points_.size() - 1; ++i)
        {
            std::pair<double, double> p1 = {current_path_points_[i].x,
                                            current_path_points_[i].y};
            std::pair<double, double> p2 = {current_path_points_[i + 1].x,
                                            current_path_points_[i + 1].y};
              
            // Closest point on segment to robot
            std::pair<double, double> proj =
                vector_projection(p1, p2, {robot_x, robot_y});

            // dist from point to robot
            double dx = robot_x - proj.first;
            double dy = robot_y - proj.second;
            double d = std::sqrt(dx * dx + dy * dy);

            if (d < min_dist)
            {
                min_dist = d;
                closest_idx = i;
                best_proj = proj;
            }
        }
        // Check last point too if single point path (edge case)
        if (current_path_points_.size() == 1)
        {
            closest_idx = 0;
            best_proj = {current_path_points_[0].x, current_path_points_[0].y};
        }

        // 2. Lookahead Logic
        double dist_travelled = 0.0;
        std::pair<double, double> current_pos = best_proj;

        // Search forward from closest segment
        for (size_t i = closest_idx; i < current_path_points_.size() - 1; ++i)
        {
            double next_x = current_path_points_[i + 1].x;
            double next_y = current_path_points_[i + 1].y;

            double dx = next_x - current_pos.first;
            double dy = next_y - current_pos.second;
            double dist_to_next = std::sqrt(dx * dx + dy * dy);

            if (dist_travelled + dist_to_next > lookahead_dist_)
            {
                double remain = lookahead_dist_ - dist_travelled;
                double ratio = remain / dist_to_next;
                double carrot_x = current_pos.first + ratio * dx;
                double carrot_y = current_pos.second + ratio * dy;
                return {carrot_x, carrot_y, 0.0, 0.0};
            }

            dist_travelled += dist_to_next;
            current_pos = {next_x, next_y};
        }

        return current_path_points_.back();
    }


    // === Scale down speed as robot gets close to goal ===
    double PurePursuitController::linear_slowdown(double desired_v,
                                                  double dist_to_goal)
    {
        if (slow_dist_ <= 0)
            return desired_v;
        double scale = std::min(dist_to_goal / slow_dist_, 1.0);
        return desired_v * scale;
    }

    std::pair<double, double> PurePursuitController::coordinate_transform(
        std::tuple<double, double, double> robot_pose,
        std::pair<double, double> target_point)
    {
        double rx = std::get<0>(robot_pose);
        double ry = std::get<1>(robot_pose);
        double theta = std::get<2>(robot_pose);

        double tx = target_point.first;
        double ty = target_point.second;

        double dx = tx - rx;
        double dy = ty - ry;

        double c = std::cos(theta);
        double s = std::sin(theta);

        double x_local = dx * c + dy * s;
        double y_local = -dx * s + dy * c;

        return {x_local, y_local};

      }



    // === computes pure pursuit curavture and applies angular velocty limits ===
    // kappa = 2*y / (x^2 + y^2)
    // w = v * kappa
    // keeps w close to max_w_ and reduces v if needed
    std::pair<double, double> PurePursuitController::compute_curvature_and_velocity(
        std::pair<double, double> local_point, double current_v)
    {
        double x = local_point.first;
        double y = local_point.second;

        double L2 = x * x + y * y;
        if (L2 < 0.0001)
            return {0.0, 0.0};

        double kappa = (2.0 * y) / L2;
        double w = current_v * kappa;

        // Scale v down if w exceeds max_w to prevent understeering
        if (std::abs(w) > max_w_)
        {
            w = std::copysign(max_w_, w);
            if (std::abs(kappa) > 1e-6)
            {
                current_v = std::abs(w / kappa);
            }
            else
            {
                current_v = 0.0;
            }
        }

        return {current_v, w};
    }


    // === converts (v, w) into left/right wheel velocities ===
    // Differential drive:
    // v_left  = v - w*(track_width/2)
    // v_right = v + w*(track_width/2)
    std::pair<double, double>
    PurePursuitController::differential_drive_kinematics(double v, double w)
    {
        double half_track = track_width_ / 2.0;
        double v_left = v - w * half_track;
        double v_right = v + w * half_track;
        return {v_left, v_right};
    }



    // === pipeline ===
    std::pair<double, double> PurePursuitController::compute_control(
        std::tuple<double, double, double> robot_pose,
        std::pair<double, double> lookahead_point,
        std::pair<double, double> goal_point)
    {
        // 1. Linear Slowdown
        double rx = std::get<0>(robot_pose);
        double ry = std::get<1>(robot_pose);
        double dist_to_goal = std::sqrt(std::pow(rx - goal_point.first, 2) +
                                        std::pow(ry - goal_point.second, 2));
        double v_desired = linear_slowdown(max_v_, dist_to_goal);

        // 2. Coordinate Transform
        auto local_point = coordinate_transform(robot_pose, lookahead_point);

        // 3. Pure Pursuit & Velocity Scaling
        auto [v_cmd, w_cmd] = compute_curvature_and_velocity(local_point, v_desired);

        // 4. Differential Drive Kinematics
        return differential_drive_kinematics(v_cmd, w_cmd);
    }

}
