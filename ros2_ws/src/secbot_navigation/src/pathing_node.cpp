#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "secbot_navigation/grid_map.hpp"
#include "secbot_navigation/planner_server.hpp"
#include "secbot_navigation/control/traj_controller.h"
#include "secbot_navigation/trajectory.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unordered_set>
#include <sstream>
#include <iomanip>


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
      if (!this->has_parameter("use_sim_time")) {
        this->declare_parameter("use_sim_time", false);
      }
      this->declare_parameter<std::string>("config_file", "");
      this->declare_parameter<std::string>("arena_file", "");

      // Load configs
      load_config();

      // Init the planner (brain of pathing) using grid map
      init_planner();

      // Ros interfaces publish and subscriber nodes
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // output motion commands
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PathingNode::odom_callback, this, _1)); // input robot pose
      timer_ = this->create_wall_timer(100ms, std::bind(&PathingNode::control_loop, this)); // timer at 10 Hz, controls output
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1); // publish planned path for RViz
      // debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/nav_debug", 1); // placeholder for markers
      // pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav_pose", 10); // placeholder for publishing pose
      goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/nav/goal_reached", 10); // signals when nav goal is reached
      vision_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathingNode::goal_callback, this, _1)); // dynamic goal input
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_print_timer_ = this->create_wall_timer( 250ms, std::bind(&PathingNode::print_tf_pose, this) );
      RCLCPP_INFO(this->get_logger(), "Pathing Node Initialized");
    }

  private:
    // ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; // Path publisher for RViz
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_; // MarkerArray publisher for debug visualization
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr tf_print_timer_;



    ::TrajectoryController traj_controller_;
    std::vector<::TrajectoryController::Waypoint> waypoints_;
    rclcpp::Time last_time_;
    rclcpp::Time last_odom_stamp_;

    // variables ====================================
    // helper 
    bool goal_reached_ = false;
    bool printed_waiting_for_odom_ = false;
    bool printed_waiting_for_traj_ = false;

    bool has_goal_ = false;
    double min_goal_dist_ = 0.75;      // tune
    double goal_change_thresh_ = 0.25; // tune

    bool planned_once_ = false;
    bool planned_once_goal_callback_ = false;
    bool have_last_odom_stamp_ = false;
    bool have_last_time_ = false;

    // State from odom
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool state_received_ = false;

    double robot_radius_ = 0.22;   // approx footprint radius (tune)
    double inflation_ = 0.08;      // extra safety margin

    // grid configs from "arena_layout.yaml" ==================================
    // grid
    double resolution_ = 0.25;
    int width = 0;
    int height = 0;
    // robot start/goal
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;
    // origin of field
    std::pair<double, double> origin_ = {0.0, 0.0};

    // robot speed "nav_sim.yaml"
    // trajectory
    double speed_ = 0.5;
    double lookahead = 1.0;
    int max_skip_ = 1;
    
    // PURE PURSUIT CONFIGS ==================================
    float max_v_ = 1.0f;
    float max_w_ = 1.7f;
    float slowdown_dist_ = 3.0f; 
    float pos_tol_= 0.45f;
    float advance_tol_= 0.45f;
    bool preserve_curvature_on_w_saturation_= true;
    float min_v_near_goal_= 0.15;
    
    // obstacles for printing
    struct RectObs {
      double x;
      double y;
      double w;
      double h;
    };
    std::vector<RectObs> obstacle_rects_;


    // Components =======================
    std::unique_ptr<GridMap> grid_map_; // Occupancy grid
    std::unique_ptr<PlannerServer> planner_; // Planning of D* Lite + smoothing + trajecotry conversion
    std::unique_ptr<Trajectory> current_trajectory_; // Current followed plan 



    // loads nav.yaml configs =======================
    void load_config()
    {
      RCLCPP_INFO(this->get_logger(), "--------------------");
      RCLCPP_INFO(this->get_logger(), "Starting loading config");

      std::string nav_path = this->get_parameter("config_file").as_string();
      std::string arena_path = this->get_parameter("arena_file").as_string();
      if (nav_path.empty() || arena_path.empty()) {
        RCLCPP_FATAL(this->get_logger(), "NO SET CONFIG FILES. nav='%s' arena='%s'",nav_path.c_str(), arena_path.c_str());
        throw std::runtime_error("MISSING PARAMS");
      }

      RCLCPP_INFO(this->get_logger(),"Loaded nav_path config: %s", nav_path.c_str());
      RCLCPP_INFO(this->get_logger(),"Loaded arena_path config: %s", arena_path.c_str());
      try
      {
        YAML::Node nav_cfg = YAML::LoadFile(nav_path);
        YAML::Node arena_cfg = YAML::LoadFile(arena_path);

        // === Arena Config ===
        if (arena_cfg["grid"]["width"]) width = arena_cfg["grid"]["width"].as<int>();
        if (arena_cfg["grid"]["height"]) height = arena_cfg["grid"]["height"].as<int>();
        if (arena_cfg["grid"]["resolution"]) resolution_ = arena_cfg["grid"]["resolution"].as<double>();
        if (arena_cfg["origin"]["x"]) origin_.first = arena_cfg["origin"]["x"].as<double>();
        if (arena_cfg["origin"]["y"]) origin_.second = arena_cfg["origin"]["y"].as<double>();

        // === Goal ===
        if (arena_cfg["goal"]["x"]) goal_x_ = arena_cfg["goal"]["x"].as<double>();
        if (arena_cfg["goal"]["y"]) goal_y_ = arena_cfg["goal"]["y"].as<double>();

        // === Start ===
        if (arena_cfg["start"]["x"]) start_x_ = arena_cfg["start"]["x"].as<double>();
        if (arena_cfg["start"]["y"]) start_y_ = arena_cfg["start"]["y"].as<double>();

        // === Nav Config ===
        if (nav_cfg["trajectory"]["speed"]) speed_ = nav_cfg["trajectory"]["speed"].as<double>(); // add speed
        if (nav_cfg["trajectory"]["lookahead_distance"]) lookahead = nav_cfg["trajectory"]["lookahead_distance"].as<double>(); // add lookahead distance
        if (nav_cfg["smoothing"]["max_skip"]) max_skip_ = nav_cfg["smoothing"]["max_skip"].as<int>(); // add max skip

        ::TrajectoryController::Config tc{};
        tc.lookahead_dist = static_cast<float>(lookahead);
        tc.cruise_v       = static_cast<float>(speed_);

        // Pick sane limits (or read from YAML if you add keys)
        tc.max_v = std::max(tc.cruise_v, max_v_);   // or e.g. 1.0f
        tc.max_w = max_w_;                         // tune as needed
        tc.slowdown_dist = slowdown_dist_;                 // meters; tune
        tc.pos_tol = pos_tol_;                       // match your old goal radius
        tc.advance_tol = advance_tol_;                  // how close before advancing waypoint
        tc.preserve_curvature_on_w_saturation = preserve_curvature_on_w_saturation_;
        tc.min_v_near_goal = min_v_near_goal_;
        
        if (nav_cfg["pure_pursuit"]["max_v"]) tc.max_v = std::max(tc.cruise_v, nav_cfg["pure_pursuit"]["max_v"].as<float>()); // add speed
        if (nav_cfg["pure_pursuit"]["max_w"]) tc.max_w = nav_cfg["pure_pursuit"]["max_w"].as<float>(); // add lookahead distance
        if (nav_cfg["pure_pursuit"]["slowdown_dist"]) tc.slowdown_dist = nav_cfg["pure_pursuit"]["slowdown_dist"].as<float>(); // add max skip
        if (nav_cfg["pure_pursuit"]["pos_tol"]) tc.pos_tol = nav_cfg["pure_pursuit"]["pos_tol"].as<float>(); // add speed
        if (nav_cfg["pure_pursuit"]["advance_tol"]) tc.advance_tol = nav_cfg["pure_pursuit"]["advance_tol"].as<float>(); // add lookahead distance
        if (nav_cfg["pure_pursuit"]["preserve_curvature_on_w_saturation"]) tc.preserve_curvature_on_w_saturation = nav_cfg["pure_pursuit"]["preserve_curvature_on_w_saturation"].as<bool>(); // add max skip
        if (nav_cfg["pure_pursuit"]["min_v_near_goal"]) tc.min_v_near_goal = nav_cfg["pure_pursuit"]["min_v_near_goal"].as<float>(); // add max skip
        traj_controller_.configure(tc);

        
        // === Init Grid ===
        RCLCPP_INFO(this->get_logger(), "Grid loaded: width=%d height=%d res=%.3f origin=(%.2f,%.2f)",
            width, height, resolution_, origin_.first, origin_.second);

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


    bool cell_in_bounds(int r, int c) const {
      return grid_map_ &&
            r >= 0 && r < grid_map_->get_rows() &&
            c >= 0 && c < grid_map_->get_cols();
    }

    void init_planner()
    {
      if (!grid_map_) {
        RCLCPP_FATAL(this->get_logger(), "grid_map_ non existent. mostlikely configs");
        throw std::runtime_error("INITIALZIE grid_map_");
      }
      PlannerConfig p_cfg;
      p_cfg.speed = speed_;
      p_cfg.max_skip = max_skip_;
      planner_ = std::make_unique<PlannerServer>(*grid_map_, p_cfg);

      auto [gr, gc] = world_to_cell(goal_x_, goal_y_);

      if (!cell_in_bounds(gr, gc)) {
        RCLCPP_ERROR(this->get_logger(),
          "GOAL OUT OF BOUNDS: world=(%.2f,%.2f) cell=(%d,%d) grid=(%d,%d) origin=(%.2f,%.2f) res=%.2f",
          goal_x_, goal_y_, gr, gc,
          grid_map_->get_rows(), grid_map_->get_cols(),
          origin_.first, origin_.second, resolution_);
        return;
      }

      planner_->set_goal(GridMap::Cell{gr, gc});
    }
    




    // === computes inital plam from YAML start to goal ===
    void compute_plan()
    {
      RCLCPP_INFO(this->get_logger(), "--------------------");
      RCLCPP_INFO(this->get_logger(), "Computing initial path");

      auto [sr, sc] = world_to_cell(start_x_, start_y_);
      if (!cell_in_bounds(sr, sc)) {
        RCLCPP_ERROR(this->get_logger(),
          "START OUT OF BOUNDS: world=(%.2f,%.2f) cell=(%d,%d)", start_x_, start_y_, sr, sc);
        return;
      }

      RCLCPP_INFO(this->get_logger(),
        "Planning: start world=(%.2f,%.2f) cell=(%d,%d) goal world=(%.2f,%.2f)",
        start_x_, start_y_, sr, sc, goal_x_, goal_y_);


      // created plan
      auto trajectory_plan = planner_->compute_plan(GridMap::Cell{
        sr, sc
      });


      if (trajectory_plan)
      {
        current_trajectory_ = std::make_unique<Trajectory>();

        for (const auto& cell : *trajectory_plan) {
          auto [wx, wy] = cell_to_world(cell.x, cell.y);  // adjust field names
          current_trajectory_->add_point(wx, wy, speed_);
        }
        waypoints_.clear();
        // waypoints_.reserve(trajectory_plan->size()); // if Trajectory supports size()

        for (const auto& p : *current_trajectory_) {
          ::TrajectoryController::Waypoint wp{};
          wp.x = static_cast<float>(p.x);
          wp.y = static_cast<float>(p.y);
          waypoints_.push_back(wp);
        }


        if (waypoints_.empty()) {
          traj_controller_.clearTrajectory();
        } else {
          traj_controller_.setTrajectory(waypoints_.data(), waypoints_.size());
        }


        have_last_time_ = false;


        RCLCPP_INFO(this->get_logger(), "Path Computed Successfully");
        this->publish_and_print_path(*current_trajectory_); // visualize path
        print_grid_with_path(*current_trajectory_);

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
      if (!state_received_) {
        RCLCPP_WARN(this->get_logger(), "Ignoring /goal_pose: no /odom yet");
        return;
      }
      if (!planned_once_goal_callback_){
        const double gx = msg->pose.position.x;
        const double gy = msg->pose.position.y;

        const double d_robot = std::hypot(gx - robot_x_, gy - robot_y_);
        if (d_robot < min_goal_dist_) {
          RCLCPP_WARN(this->get_logger(),
            "Ignoring /goal_pose: too close to robot (d=%.2fm < %.2fm). gx=%.2f gy=%.2f robot=(%.2f,%.2f)",
            d_robot, min_goal_dist_, gx, gy, robot_x_, robot_y_);
          return;
        }

        if (has_goal_) {
          const double d_change = std::hypot(gx - goal_x_, gy - goal_y_);
          if (d_change < goal_change_thresh_) {
            return; // tiny jitter -> don't replan
          }
        }

        goal_x_ = gx;
        goal_y_ = gy;
        has_goal_ = true;
        planned_once_goal_callback_ = true;
        RCLCPP_INFO(this->get_logger(), "Accepted new goal: (%.2f, %.2f) d_robot=%.2f", goal_x_, goal_y_, d_robot);
        init_planner();
        compute_plan();  // or whatever your plan trigger is
      }
    }








    // ODOM TO UPDATE ROBOT POSE =====================================
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

      last_odom_stamp_ = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
      have_last_odom_stamp_ = true;

      // if (!planned_once_) {
      //   // Plan from the REAL robot pose, not YAML start

      //   RCLCPP_INFO(this->get_logger(), "ROBOT POSITION (%.2f, %.2f) VS START POSITION (%.2f, %.2f)", robot_x_, robot_y_, start_x_, start_y_);
      //   start_x_ = robot_x_;
      //   start_y_ = robot_y_;

      //   init_planner();
      //   compute_plan();

      //   planned_once_ = true;
      //   RCLCPP_INFO(this->get_logger(),
      //     "Planned from first odom pose (%.2f, %.2f) to goal (%.2f, %.2f)",
      //     start_x_, start_y_, goal_x_, goal_y_);
      // }
    }
    
    // Control loop ===============================================
    void control_loop()
    {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "control_loop running");

      if (!state_received_ || !current_trajectory_) {
        if (!state_received_) {
          if (!printed_waiting_for_odom_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for /odom... (control loop paused)");
            printed_waiting_for_odom_ = true;
          } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Still waiting for /odom...");
          }
        }

        if (!current_trajectory_) {
          if (!printed_waiting_for_traj_) {
            RCLCPP_WARN(this->get_logger(), "No trajectory available yet (plan not computed)");
            printed_waiting_for_traj_ = true;
          } else {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Still no trajectory...");
          }
        }
        return;
      }

      // if we get here, we are good:
      printed_waiting_for_odom_ = false;
      printed_waiting_for_traj_ = false;

      if (have_last_odom_stamp_) {
        double odom_age = (this->now() - last_odom_stamp_).seconds();
        if (odom_age > 0.5) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "ODOM STALE: age=%.3fs", odom_age);
        }
      }

      // --- dt computation ---
      auto now = this->now();
      float dt = 0.1f; // fallback
      if (have_last_time_) {
        dt = static_cast<float>((now - last_time_).seconds());
      }
      last_time_ = now;
      have_last_time_ = true;

      // --- call lead's controller ---
      ::TrajectoryController::Pose2D pose{};
      pose.x = static_cast<float>(robot_x_);
      pose.y = static_cast<float>(robot_y_);
      pose.theta = static_cast<float>(robot_theta_);

      if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.25f) {
        publisher_->publish(geometry_msgs::msg::Twist()); // STOP
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Bad dt=%.3f, stopping", dt);
        return;
      }

      ::TrajectoryController::Command cmd = traj_controller_.update(pose, dt);


      if (cmd.finished){
        publisher_->publish(geometry_msgs::msg::Twist());
        // If controller says finished, publish goal reached once
        if (!goal_reached_) {
          goal_reached_ = true;
          std_msgs::msg::Bool reached;
          reached.data = true;
          goal_reached_pub_->publish(reached);
          RCLCPP_INFO(this->get_logger(), "TrajectoryController reports finished. Stopping.");
        }
        return;
      }
      // Publish v,w directly as Twist
      geometry_msgs::msg::Twist out;
      out.linear.x = cmd.v;
      out.angular.z = cmd.w;
      publisher_->publish(out);


      // Optional: debug
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "traj_cmd v=%.3f w=%.3f lookahead=(%.2f,%.2f)",
        cmd.v, cmd.w, cmd.lookahead_x, cmd.lookahead_y
      );

    }






    // PRINT THE INFO ===============================================
    // THIS PART IS ONLY FOR DEBUGGING AND FIGURING OUT UNDERLYING ISSUES. THIS HAS NO REAL VALUE BUT TO JUST DEBUG. CAN BE REMOVED LATER IN THE FINAL UPLOAD
    // THIS PART WAS GENERATED TO MAKE MY LIFE EASIER. THIS WILL BE REMOVED LATER
    // === Convert a Trajectory into a nav_msgs/Path message for RViz ===
    nav_msgs::msg::Path to_path_msg(const Trajectory &traj)
    {
      nav_msgs::msg::Path path;

      // Use the node clock (respects use_sim_time)
      path.header.stamp = this->get_clock()->now();

      // Keep frame consistent with what you're already using elsewhere.
      path.header.frame_id = "odom";

      path.poses.reserve(traj.get_points().size());

      for (const auto &p : traj.get_points())
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;

        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.position.z = 0.0;

        // Identity orientation (RViz is fine with this)
        ps.pose.orientation.x = 0.0;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = 0.0;
        ps.pose.orientation.w = 1.0;

        path.poses.push_back(std::move(ps));
      }

      return path;
    }


    void publish_and_print_path(const Trajectory &traj)
    {
      // 1) Publish for RViz
      auto path_msg = to_path_msg(traj);
      path_pub_->publish(path_msg);

      // 2) Print a compact summary + first/last points
      const auto &pts = traj.get_points();
      if (pts.empty())
      {
        RCLCPP_WARN(this->get_logger(), "=== GENERATED PATH: EMPTY ===");
        return;
      }

      int collide_count = 0;
      if (grid_map_)
      {
        for (const auto &p : pts)
        {
          auto [r, c] = world_to_cell(p.x, p.y);
          if (r >= 0 && r < grid_map_->get_rows() && c >= 0 && c < grid_map_->get_cols())
          {
            if (!grid_map_->is_free(GridMap::Cell{r, c}))
              collide_count++;
          }
        }
      }

      RCLCPP_INFO(this->get_logger(),
        "=== GENERATED PATH: N=%zu  start=(%.2f,%.2f)  end=(%.2f,%.2f)  collisions=%d ===",
        pts.size(),
        pts.front().x, pts.front().y,
        pts.back().x,  pts.back().y,
        collide_count
      );

      // 3) Print points (throttled-ish by limiting count)
      // Print all if it's small, otherwise print first 30 and last 10.
      const size_t N = pts.size();
      const size_t head = std::min<size_t>(30, N);
      const size_t tail = (N > 40) ? 10 : 0;

      for (size_t i = 0; i < head; ++i)
      {
        const auto &p = pts[i];
        RCLCPP_INFO(this->get_logger(), "Point %zu: (%.2f, %.2f) v=%.2f", i, p.x, p.y, p.v);
      }

      if (tail > 0)
      {
        RCLCPP_INFO(this->get_logger(), "... (%zu points omitted) ...", N - head - tail);
        for (size_t i = N - tail; i < N; ++i)
        {
          const auto &p = pts[i];
          RCLCPP_INFO(this->get_logger(), "Point %zu: (%.2f, %.2f) v=%.2f", i, p.x, p.y, p.v);
        }
      }

      RCLCPP_INFO(this->get_logger(), "=========================================");
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



    void print_grid_with_path(const Trajectory& traj)
    {
      if (!grid_map_)
      {
        RCLCPP_WARN(this->get_logger(), "print_grid_with_path: grid_map_ is null");
        return;
      }

      const int rows = grid_map_->get_rows();
      const int cols = grid_map_->get_cols();

      // Pack (r,c) -> single key for hash set
      auto pack = [cols](int r, int c) -> long long {
        return static_cast<long long>(r) * static_cast<long long>(cols) + c;
      };

      // Collect path cells into a fast lookup set
      std::unordered_set<long long> path_cells;
      path_cells.reserve(traj.get_points().size() * 2 + 64);

      for (const auto& p : traj.get_points())
      {
        auto [r, c] = world_to_cell(p.x, p.y);
        if (r >= 0 && r < rows && c >= 0 && c < cols)
          path_cells.insert(pack(r, c));
      }

      // Start/Goal/Robot cells (robot marker is optional but very helpful)
      auto [sr, sc] = world_to_cell(start_x_, start_y_);
      auto [gr, gc] = world_to_cell(goal_x_, goal_y_);
      auto [rr, rc] = world_to_cell(robot_x_, robot_y_);

      std::ostringstream out;
      out << "\n========== GRID VIEW (top = +y) ==========\n";
      out << "Legend: . free, # obstacle, * path, X path-in-obstacle, S start, G goal, R robot\n";
      out << "Grid: " << rows << " rows x " << cols << " cols, res=" << resolution_
          << " origin=(" << origin_.first << "," << origin_.second << ")\n\n";

      for (int r = rows - 1; r >= 0; --r)
      {
        out << std::setw(3) << r << " ";

        for (int c = 0; c < cols; ++c)
        {
          const bool obstacle = !grid_map_->is_free(GridMap::Cell{r, c});
          const bool on_path  = (path_cells.find(pack(r, c)) != path_cells.end());

          char ch = '.';
          if (obstacle && on_path) ch = 'X';
          else if (obstacle)       ch = '#';
          else if (on_path)        ch = '*';

          // Overrides (most important last)
          if (r == sr && c == sc) ch = 'S';
          if (r == gr && c == gc) ch = 'G';
          if (r == rr && c == rc) ch = 'R';

          out << ch;
        }
        out << "\n";
      }

      // Simple, readable axis: mark every 5 columns
      out << "    ";
      for (int c = 0; c < cols; ++c)
        out << ((c % 5 == 0) ? '|' : ' ');
      out << "\n";

      out << "    ";
      for (int c = 0; c < cols; ++c)
      {
        if (c % 5 == 0)
          out << static_cast<char>('0' + (c % 10));   // ones digit
        else
          out << ' ';
      }
      out << "\n";

      out << "==========================================\n";

      RCLCPP_INFO(this->get_logger(), "%s", out.str().c_str());
    }

    void print_tf_pose()
    {
      try {
        // Try common frame pairs. One of these will match your sim.
        const std::string parent = "odom";
        const std::string child  = "base_link";

        auto tf = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);

        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;

        tf2::Quaternion q(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 250,
          "TF pose (%s->%s): x=%.3f y=%.3f yaw=%.3f",
          parent.c_str(), child.c_str(), x, y, yaw
        );
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF lookup failed: %s", e.what()
        );
      }
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
