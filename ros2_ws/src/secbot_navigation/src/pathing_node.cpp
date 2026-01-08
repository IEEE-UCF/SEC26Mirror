#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "rclcpp/rclcpp.hpp"
#include "secbot_navigation/grid_map.hpp"
#include "secbot_navigation/planner_server.hpp"
#include "secbot_navigation/trajectory.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot_navigation
{

  // === ROS2 Node for planning + following a path ===
  class PathingNode : public rclcpp::Node
  {
  public:
  // === Node constructor ===
    PathingNode() : Node("pathing_node")
    {
      // 1. Declare Parameters
      if (!this->has_parameter("use_sim_time"))
      {
        this->declare_parameter("use_sim_time", false);
      }
      this->declare_parameter("config_file", "nav.yaml");
      this->declare_parameter("arena_file", "arena_layout.yaml");

      // 2. Load Configuration
      load_config();

      // 3. Initialize Planner
      init_planner();

      // 5. Setup ROS Interfaces
      publisher_ =
          this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/odom", 10, std::bind(&PathingNode::odom_callback, this, _1));

      // Control Loop Timer (10 Hz if configured)
      timer_ = this->create_wall_timer(
          100ms, std::bind(&PathingNode::control_loop, this));
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
      debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/nav_debug", 1);
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav_pose", 10);

      // 4. Compute Initial Plan
      compute_plan();
      RCLCPP_INFO(this->get_logger(), "Pathing Node Initialized");
    }

  private:
    // ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // Path publisher for RViz
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_; // MarkerArray publisher for debug visualization
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_; 

    rclcpp::TimerBase::SharedPtr timer_;
    // === helpers ===
    void log_state(const char *tag)
    {
      RCLCPP_INFO(this->get_logger(), tag);
    }

    void diagnose(const char *tag)
    {
      RCLCPP_INFO(
          this->get_logger(),
          "[%s] pose=(%.2f, %.2f) yaw=%.2f goal=(%.2f, %.2f) odom=%d path=%d",
          tag,
          robot_x_, robot_y_, robot_theta_,
          goal_x_, goal_y_,
          state_received_ ? 1 : 0,
          current_trajectory_ ? 1 : 0);
    }

    // === Callback to handle new goals from vision ===
    void vision_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      double new_x = msg->pose.position.x;
      double new_y = msg->pose.position.y;

      RCLCPP_INFO(this->get_logger(), "Vision Detected Goal: (%f, %f)", new_x,
                  new_y);

      // === Update Goal ===
      goal_x_ = new_x;
      goal_y_ = new_y;

      // === Trigger Re-planning ===
      int gr = static_cast<int>((goal_y_ - origin_.second) / resolution_);
      int gc = static_cast<int>((goal_x_ - origin_.first) / resolution_);
      planner_->set_goal({gr, gc});

      // === Recompute path from current robot position ===
      int sr = static_cast<int>((robot_y_ - origin_.second) / resolution_);
      int sc = static_cast<int>((robot_x_ - origin_.first) / resolution_);

      if (!state_received_){}

      auto traj_opt = planner_->compute_plan({sr, sc});
      if (traj_opt)
      {
        current_trajectory_ = std::make_unique<Trajectory>(*traj_opt);
        controller_->set_path(*current_trajectory_);
        RCLCPP_INFO(this->get_logger(), "Replanned path to vision target");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Failed to plan path to vision target");
      }
    }

    // Components
    std::unique_ptr<GridMap> grid_map_;
    std::unique_ptr<PlannerServer> planner_;
    std::unique_ptr<PurePursuitController> controller_;
    std::unique_ptr<Trajectory> current_trajectory_;

    // State
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool state_received_ = false;

    // Configs
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double speed_ = 0.5;
    std::pair<double, double> origin_ = {0.0, 0.0};
    double resolution_ = 0.1;
    int max_skip_ = 1;


    // === loads nav.yaml configs ===
    void load_config()
    {
      log_state("Starting loading config");
      std::string pkg_share =
          ament_index_cpp::get_package_share_directory("secbot_navigation");

      std::string nav_file = this->get_parameter("config_file").as_string();
      std::string arena_file = this->get_parameter("arena_file").as_string();

      std::string nav_path = pkg_share + "/config/" + nav_file;
      std::string arena_path = pkg_share + "/config/" + arena_file;

      RCLCPP_INFO(this->get_logger(), "Loading configs: %s, %s", nav_path.c_str(),
                  arena_path.c_str());

      try
      {
        YAML::Node nav_cfg = YAML::LoadFile(nav_path);
        YAML::Node arena_cfg = YAML::LoadFile(arena_path);

        // === Arena Config ===
        int width = arena_cfg["grid"]["width"].as<int>();
        int height = arena_cfg["grid"]["height"].as<int>();
        resolution_ = arena_cfg["grid"]["resolution"].as<double>();
        origin_.first = arena_cfg["origin"]["x"].as<double>();
        origin_.second = arena_cfg["origin"]["y"].as<double>();

        // === Init Grid ===
        std::vector<std::vector<int>> grid_data(height,
                                                std::vector<int>(width, 0));
        grid_map_ = std::make_unique<GridMap>(grid_data);

        // === Obstacles ===
        const YAML::Node &obstacles = arena_cfg["obstacles"];
        for (std::size_t i = 0; i < obstacles.size(); ++i)
        {
          const YAML::Node &obs = obstacles[i];
          if (obs["type"].as<std::string>() == "rectangle")
          {
            mark_rectangle_obstacle(obs);
          }
        }

        // === Goal ===
        goal_x_ = arena_cfg["goal"]["x"].as<double>();
        goal_y_ = arena_cfg["goal"]["y"].as<double>();

        // === Start ===
        start_x_ = arena_cfg["start"]["x"].as<double>();
        start_y_ = arena_cfg["start"]["y"].as<double>();

        // === Nav Config ===
        speed_ = 0.5;
        double lookahead = 1.0;
        max_skip_ = 1;
        if (nav_cfg["trajectory"]["speed"])
          speed_ = nav_cfg["trajectory"]["speed"].as<double>();
        if (nav_cfg["trajectory"]["lookahead_distance"])
          lookahead = nav_cfg["trajectory"]["lookahead_distance"].as<double>();
        if (nav_cfg["smoothing"]["max_skip"])
          max_skip_ = nav_cfg["smoothing"]["max_skip"].as<double>();

        // === Init Controller ===
        controller_ = std::make_unique<PurePursuitController>(0.5, lookahead,
                                                              speed_, 2.0, 2.0);
        log_state("STOPPED loading config");
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Config Load Error: %s", e.what());
      }
    }
    // === Marks obstacle from YAML into GridMap ===
    void mark_rectangle_obstacle(const YAML::Node &obs)
    {
      double x0 = obs["x"].as<double>();
      double y0 = obs["y"].as<double>();
      double w = obs["width"].as<double>();
      double h = obs["height"].as<double>();

      double x = x0;
      while (x < x0 + w)
      {
        double y = y0;
        while (y < y0 + h)
        {
          int r = static_cast<int>((y - origin_.second) / resolution_);
          int c = static_cast<int>((x - origin_.first) / resolution_);
          if (r >= 0 && r < grid_map_->get_rows() && c >= 0 &&
              c < grid_map_->get_cols())
          {
            grid_map_->set_obstacle({r, c});
          }
          y += resolution_;
        }
        x += resolution_;
      }
    }
    // === initializes plannerserver and sets goal ===
    void init_planner()
    {

      log_state("STARTED init planner");
      PlannerConfig p_cfg;
      p_cfg.speed = speed_;
      p_cfg.max_skip = max_skip_;
      planner_ = std::make_unique<PlannerServer>(*grid_map_, p_cfg);

      // Set Goal
      int gr = static_cast<int>((goal_y_ - origin_.second) / resolution_);
      int gc = static_cast<int>((goal_x_ - origin_.first) / resolution_);
      planner_->set_goal({gr, gc});

      log_state("STOPPED init planner");
    }
    
    // === computes inital plam from YAML start to goal ===
    void compute_plan()
    {

      int sr = static_cast<int>((start_y_ - origin_.second) / resolution_);
      int sc = static_cast<int>((start_x_ - origin_.first) / resolution_);

      auto traj_opt = planner_->compute_plan({sr, sc});
      if (traj_opt)
      {
        current_trajectory_ = std::make_unique<Trajectory>(*traj_opt);
        controller_->set_path(*current_trajectory_);
        RCLCPP_INFO(this->get_logger(), "Path Computed Successfully");
        this->publish_and_print_path(*current_trajectory_); // visualize path
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute initial path");
      }
    }


    // === Odom callback updates robot pose ===
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      robot_x_ = msg->pose.pose.position.x;
      robot_y_ = msg->pose.pose.position.y;

      tf2::Quaternion q(
          msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      robot_theta_ = yaw;

      state_received_ = true;
    }
    
    // // === main control loop // ===
    void control_loop()
    {
      if (!state_received_ || !current_trajectory_)
        return;

      // Check goal
      double dx = goal_x_ - robot_x_;
      double dy = goal_y_ - robot_y_;
      if (std::sqrt(dx * dx + dy * dy) < 0.2)
      {
        // Stop
        auto stop_msg = geometry_msgs::msg::Twist();
        publisher_->publish(stop_msg);
        return;
      }

      // Pure Pursuit Control
      // Find Carrot
      TrajectoryPoint carrot =
          controller_->find_lookahead_point(robot_x_, robot_y_);

      // Compute Control
      auto wheel_speeds =
          controller_->compute_control({robot_x_, robot_y_, robot_theta_},
                                       {carrot.x, carrot.y}, {goal_x_, goal_y_});

      // Convert Wheel Speeds to Twist (v, w)
      double vl = wheel_speeds.first;
      double vr = wheel_speeds.second;
      double track = controller_->get_track_width();

      double v = (vr + vl) / 2.0;
      double w = (vr - vl) / track;

      auto cmd_msg = geometry_msgs::msg::Twist();
      cmd_msg.linear.x = v;
      cmd_msg.angular.z = w;
      publisher_->publish(cmd_msg);
    }

    // === Convert a Trajectory into a nav_msgs/Path message for RViz ===
    nav_msgs::msg::Path to_path_msg(const Trajectory &traj)
    {
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      path.header.stamp = this->now();

      for (const auto &p : traj)
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.orientation.w = 1.0; // no yaw stored here
        path.poses.push_back(ps);
      }
      return path;
    }
    // === Publishes the planned path to RViz and prints it ===
    void publish_and_print_path(const Trajectory &traj)
    {
      // 1. publish for rviz
      auto path_msg = to_path_msg(traj);
      path_pub_->publish(path_msg);

      // 2. print to console
      RCLCPP_INFO(this->get_logger(), "=== GENERATED PATH SEQP ===");
      int idx = 0;
      for (const auto &p : traj)
      {
        RCLCPP_INFO(this->get_logger(), "Point %d: (%.2f, %.2f)", idx++, p.x, p.y);
      }
      RCLCPP_INFO(this->get_logger(), "===========================");
    }
  };

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot_navigation::PathingNode>());
  rclcpp::shutdown();
  return 0;
}
