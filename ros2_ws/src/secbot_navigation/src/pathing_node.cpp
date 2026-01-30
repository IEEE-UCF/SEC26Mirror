#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
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
    PathingNode() : Node("pathing_node")
    {
      // Get parameters from launch.py file
      if (!this->has_parameter("use_sim"))
      { this->declare_parameter("use_sim", false); }
      this->declare_parameter("config_file", "nav.yaml");
      this->declare_parameter("arena_file", "arena_layout.yaml");

      // Load configs
      load_config();

      // Init the planner (brain of pathing) using grid map
      init_planner();

      // Ros interfaces publish and subscriber nodes
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // output motion commands
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PathingNode::odom_callback, this, _1)); // input robot pose
      timer_ = this->create_wall_timer(100ms, std::bind(&PathingNode::control_loop, this)); // timer at 10 Hz, controls output
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1); // publish planned path for RViz
      debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/nav_debug", 1); // placeholder for markers
      pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav_pose", 10); // placeholder for publishing pose
      goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/nav/goal_reached", 10); // signals when nav goal is reached
      vision_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathingNode::goal_callback, this, _1)); // dynamic goal input
      // Compute Initial Plan
      // compute_plan(); // info from yaml file
      
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_reached_ = false;
    // === helper ===
    rclcpp::Clock::SharedPtr clock_{std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)};
    bool printed_waiting_for_odom_ = false;
    bool printed_waiting_for_traj_ = false;

    bool have_first_odom_ = false;
    bool planned_once_ = false;


    void info_once(const std::string& key, const std::string& msg)
    {
      // RCLCPP_*_ONCE exists, but this lets you have multiple "once" keys if you want later.
      static std::unordered_set<std::string> seen;
      if (seen.insert(key).second) {
        RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
      }
    }

    void info_throttle(const std::string& msg, int ms)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, ms, "%s", msg.c_str());
    }


    // Components
    std::unique_ptr<GridMap> grid_map_; // Occupancy grid
    std::unique_ptr<PlannerServer> planner_; // Planning of D* Lite + smoothing + trajecotry conversion
    std::unique_ptr<PurePursuitController> controller_; // pure pursuit (THE RAFEED ALGORITM)
    std::unique_ptr<Trajectory> current_trajectory_; // Current followed plan 


    struct RectObs {
      double x;
      double y;
      double w;
      double h;
    };
    std::vector<RectObs> obstacle_rects_;

    // State from odom
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool state_received_ = false;

    double robot_radius_ = 0.22;   // approx footprint radius (tune)
    double inflation_ = 0.08;      // extra safety margin

    // Configs from yaml
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    double speed_ = 0.5;
    std::pair<double, double> origin_ = {0.0, 0.0};
    double resolution_ = 0.1;
    int max_skip_ = 1;

    // === helper file path loader ===
    std::string load_filepath(const std::string& folder, const std::string& filename)
    {
      return ament_index_cpp::get_package_share_directory("secbot_navigation") + folder + filename;
    }

    // === loads nav.yaml configs ===
    void load_config()
    {
      RCLCPP_INFO(this->get_logger(), "--------------------");
      RCLCPP_INFO(this->get_logger(), "Starting loading config");
      std::string pkg_share = ament_index_cpp::get_package_share_directory("secbot_navigation");


      std::string nav_path = load_filepath("/config/", this->get_parameter("config_file").as_string());
      std::string arena_path = load_filepath("/config/", this->get_parameter("arena_file").as_string());


      RCLCPP_INFO(this->get_logger(),"Loaded nav_path config: %s", nav_path.c_str());
      RCLCPP_INFO(this->get_logger(),"Loaded arena_path config: %s", arena_path.c_str());
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
        std::vector<std::vector<int>> grid_data(height, std::vector<int>(width, 0));
        grid_map_ = std::make_unique<GridMap>(grid_data);

        // === Obstacles ===
        const YAML::Node &obstacles = arena_cfg["obstacles"];
        for (std::size_t i = 0; i < obstacles.size(); ++i)
        {
          const YAML::Node &obs = obstacles[i];
          if (obs["type"].as<std::string>() == "rectangle")
          {
            mark_rectangle_obstacle(obs); // generate obstacles
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
        if (nav_cfg["trajectory"]["speed"]) speed_ = nav_cfg["trajectory"]["speed"].as<double>(); // add speed
        if (nav_cfg["trajectory"]["lookahead_distance"]) lookahead = nav_cfg["trajectory"]["lookahead_distance"].as<double>(); // add lookahead distance
        if (nav_cfg["smoothing"]["max_skip"]) max_skip_ = nav_cfg["smoothing"]["max_skip"].as<int>(); // add max skip

        // === Init Controller ===
        controller_ = std::make_unique<PurePursuitController>(0.5, lookahead, speed_, 2.0, 2.0);
        RCLCPP_INFO(this->get_logger(), "Configs loaded successfully");
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Config Load Error: %s", e.what());
      }
    }

    // === Marks obstacle from YAML into GridMap ===
    void mark_rectangle_obstacle(const YAML::Node &obs)
    {
      

      // const double pad = robot_radius_ + inflation_;
      // double x = obs["x"].as<double>();
      // const double x_out = x + obs["width"].as<double>() + pad;
      // const double x_in = x - pad;

      // double y = obs["y"].as<double>();
      // const double y_out = y + obs["height"].as<double>() + pad;
      // const double y_in = y - pad;
      // while (x < x_out && x > x_in) // prevents all logic if obstacle is out of bounds on the x axis
      // {
        
      //   while (y < y_out && y > y_in) // prevents all logic if obstacle is out of bounds on the y axis
      //   {
      //     int r = static_cast<int>((y - origin_.second) / resolution_); // (y - origin_y) / resolution      (THIS RENDERS THE OBSTACLE ON THE Y AXIS)
      //     int c = static_cast<int>((x - origin_.first) / resolution_); // (x - origin_x) / resolution      (THIS RENDERS THE OBSTACLE ON THE X AXIS)

      //     if (r >= 0 && r < grid_map_->get_rows() && c >= 0 && c < grid_map_->get_cols()) // if the obstacle is within bounds
      //     { grid_map_->set_obstacle({r, c}); } // sets the new obstacle
      //     y += resolution_; // jump to the next y in real world coordinates
      //   }
      //   x += resolution_; // jump to the next x in real world coordinates
      // }

      const double x0 = obs["x"].as<double>();
      const double y0 = obs["y"].as<double>();
      const double w  = obs["width"].as<double>();
      const double h  = obs["height"].as<double>();

      // Inflate obstacle by robot footprint + safety margin
      const double pad = robot_radius_ + inflation_;

      const double x_min = x0 - pad;
      const double x_max = x0 + w + pad;
      const double y_min = y0 - pad;
      const double y_max = y0 + h + pad;

      for (double x = x_min; x <= x_max; x += resolution_)
      {
        for (double y = y_min; y <= y_max; y += resolution_)
        {
          int r = static_cast<int>((y - origin_.second) / resolution_);
          int c = static_cast<int>((x - origin_.first) / resolution_);

          if (r >= 0 && r < grid_map_->get_rows() && c >= 0 && c < grid_map_->get_cols())
          {
            grid_map_->set_obstacle({r, c});
          }
        }
      }
      obstacle_rects_.push_back(RectObs{x0, y0, w, h});
    }

    
    bool cell_is_obstacle_via_yaml(int r, int c) const
    {
      auto [wx, wy] = cell_to_world(r, c); // center of the cell

      const double pad = robot_radius_ + inflation_;

      for (const auto& o : obstacle_rects_)
      {
        // rectangle spans [x, x+w] and [y, y+h]
        if (wx >= (o.x - pad) && wx <= (o.x + o.w + pad) &&
            wy >= (o.y - pad) && wy <= (o.y + o.h + pad))
        {
          return true;
        }
      }
      return false;
    }



    // === initializes plannerserver and sets goal ===
    void init_planner()
    {
      PlannerConfig p_cfg;
      p_cfg.speed = speed_;
      p_cfg.max_skip = max_skip_;
      planner_ = std::make_unique<PlannerServer>(*grid_map_, p_cfg);

      // Set Goal
      int gr = static_cast<int>((goal_y_ - origin_.second) / resolution_); // Sets the goal on map using resolution to guide it
      int gc = static_cast<int>((goal_x_ - origin_.first) / resolution_);
      planner_->set_goal(GridMap::Cell{gr, gc});

    }
    
    // === computes inital plam from YAML start to goal ===
    void compute_plan()
    {
      RCLCPP_INFO(this->get_logger(), "--------------------");
      RCLCPP_INFO(this->get_logger(), "Computing initial path");
      // created plan
      auto trajectory_plan = planner_->compute_plan(GridMap::Cell{
        static_cast<int>((start_y_ - origin_.second) / resolution_), 
        static_cast<int>((start_x_ - origin_.first) / resolution_)
      });


      if (trajectory_plan)
      {
        current_trajectory_ = std::make_unique<Trajectory>();

        for (const auto& cell : *trajectory_plan) {
          auto [wx, wy] = cell_to_world(cell.x, cell.y);  // adjust field names
          current_trajectory_->add_point(wx, wy, speed_);
        }
        controller_->set_path(*current_trajectory_);
        RCLCPP_INFO(this->get_logger(), "Path Computed Successfully");
        this->publish_and_print_path(*current_trajectory_); // visualize path
        // print_grid_with_path(*current_trajectory_);

        RCLCPP_INFO(this->get_logger(), "Visualized path");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to compute initial path");
      }


    }


    // Dynamic goal callback from /goal_pose
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "--------------------");
      RCLCPP_INFO(this->get_logger(), "Received new goal_pose");
      goal_x_ = msg->pose.position.x;
      goal_y_ = msg->pose.position.y;
      goal_reached_ = false;

      // Replan from current robot position to new goal
      start_x_ = robot_x_;
      start_y_ = robot_y_;
      init_planner();
      compute_plan();
      RCLCPP_INFO(this->get_logger(), "Replanned to goal (%.2f, %.2f) from (%.2f, %.2f)", goal_x_, goal_y_, robot_x_, robot_y_);
    }

    // === Odom callback updates robot pose ===
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) // uses robot pose from odometry
    {
      // converts quaternion orientation to yaw using tf2
      tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      robot_x_ = msg->pose.pose.position.x;
      robot_y_ = msg->pose.pose.position.y;
      robot_theta_ = yaw;
      state_received_ = true; // so control loop can run
      if (!planned_once_) {
        // Plan from the REAL robot pose, not YAML start
        start_x_ = robot_x_;
        start_y_ = robot_y_;

        init_planner();
        compute_plan();

        planned_once_ = true;
        RCLCPP_INFO(this->get_logger(),
          "Planned from first odom pose (%.2f, %.2f) to goal (%.2f, %.2f)",
          start_x_, start_y_, goal_x_, goal_y_);
      }
    }
    
    // === main control loop ===
    void control_loop()
    {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 2000, "control_loop running");

      if (!state_received_ || !current_trajectory_) {
        if (!state_received_) {
          if (!printed_waiting_for_odom_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /odom... (control loop paused)");
            printed_waiting_for_odom_ = true;
          } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 2000, "Still waiting for /odom...");
          }
        }

        if (!current_trajectory_) {
          if (!printed_waiting_for_traj_) {
            RCLCPP_WARN(this->get_logger(), "No trajectory available yet (plan not computed)");
            printed_waiting_for_traj_ = true;
          } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *clock_, 2000, "Still no trajectory...");
          }
        }
        return;
      }

      // if we get here, we are good:
      printed_waiting_for_odom_ = false;
      printed_waiting_for_traj_ = false;


      double dist = std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_);

      if (dist < 0.2) {
        publisher_->publish(geometry_msgs::msg::Twist());
        if (!goal_reached_) {
          goal_reached_ = true;
          RCLCPP_INFO(this->get_logger(), "Goal reached (dist=%.2f). Stopping.", dist);
        }
        return;
      }

      // === Pure Pursuit Control ===
      RCLCPP_INFO(this->get_logger(), "===========================");
      RCLCPP_INFO(this->get_logger(), "Starting pure pursuit");
      
      // Find Carrot
      TrajectoryPoint carrot = controller_->find_lookahead_point(robot_x_, robot_y_);
      RCLCPP_INFO(this->get_logger(), "carrot (x=%.2f, y=%.2f), v=%.2f", carrot.x, carrot.y, carrot.v);

      // Compute wheel speeds
      auto wheel_speeds = controller_->compute_control(
        {robot_x_, robot_y_, robot_theta_},
        {carrot.x, carrot.y}, {goal_x_, goal_y_}
      );
      RCLCPP_INFO(this->get_logger(), "wheels (left=%.3f, right=%.3f)", wheel_speeds.first, wheel_speeds.second);

      // Convert Wheel Speeds to Twist (v, w)
      auto cmd_msg = geometry_msgs::msg::Twist(); // publish to /cmd_vel


      if (goal_reached_ || dist < 0.2) {
        publisher_->publish(geometry_msgs::msg::Twist()); // zero command

        if (!goal_reached_) {
          goal_reached_ = true;

          std_msgs::msg::Bool reached;
          reached.data = true;
          goal_reached_pub_->publish(reached);

          RCLCPP_INFO(this->get_logger(), "Goal reached (dist=%.2f). Stopping.", dist);
        }
        return;
      }


      double v_scale = std::clamp(dist / 1.0, 0.2, 1.0);   // 1.0m window
      cmd_msg.linear.x *= v_scale;
      cmd_msg.angular.z = std::clamp(cmd_msg.angular.z, -1.5, 1.5);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "POSE odom: (%.2f, %.2f, %.2f)  GOAL: (%.2f, %.2f)  dist=%.2f  cmd(v=%.2f,w=%.2f)",
        robot_x_, robot_y_, robot_theta_,
        goal_x_, goal_y_, dist,
        cmd_msg.linear.x, cmd_msg.angular.z
      );
      
      cmd_msg.linear.x = (wheel_speeds.second + wheel_speeds.first) / 2.0; // velocity
      cmd_msg.angular.z = (wheel_speeds.second - wheel_speeds.first) / controller_->get_track_width(); // angular velocity
            
      publisher_->publish(cmd_msg);

    }

    // === Convert a Trajectory into a nav_msgs/Path message for RViz ===
    nav_msgs::msg::Path to_path_msg(const Trajectory &traj)
    {
      nav_msgs::msg::Path path;
      path.header.frame_id = "odom";
      path.header.stamp = this->now();

      for (const auto &p : traj)
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.orientation.w = 1.0;
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
        RCLCPP_INFO(this->get_logger(), "Point %d: (%.2f, %.2f) v=%.2f", idx++, p.x, p.y, p.v);
      }
      RCLCPP_INFO(this->get_logger(), "===========================");
    }
    
    // --- convert world -> grid cell (r,c)
    std::pair<int,int> world_to_cell(double wx, double wy) const
    {
      int r = static_cast<int>(std::floor((wy - origin_.second) / resolution_));
      int c = static_cast<int>(std::floor((wx - origin_.first) / resolution_));
      return {r, c};
    }

    // --- convert grid cell -> world (center of cell)
    std::pair<double,double> cell_to_world(int r, int c) const
    {
      double wx = origin_.first + (c + 0.5) * resolution_;
      double wy = origin_.second + (r + 0.5) * resolution_;
      return {wx, wy};
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
