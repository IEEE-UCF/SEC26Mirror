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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
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
      this->declare_parameter<std::string>("planning_frame", "odom");


      // Load configs
      load_config();

      // Ros interfaces publish and subscriber nodes
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); // output motion commands
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PathingNode::odom_callback, this, _1)); // input robot pose
      timer_ = this->create_wall_timer(100ms, std::bind(&PathingNode::control_loop, this)); // timer at 10 Hz, controls output
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1); // publish planned path for RViz
      goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("/nav/goal_reached", 10); // signals when nav goal is reached
      vision_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&PathingNode::goal_callback, this, _1)); // dynamic goal input
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); // <-- CRITICAL
      tf_print_timer_ = this->create_wall_timer( 250ms, std::bind(&PathingNode::print_tf_pose, this) );
      RCLCPP_INFO(this->get_logger(), "Pathing Node Initialized");
    }

  private:
    // ROS Interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_print_timer_;
    
    // TF HELPERS ====================================
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    // variables ====================================
    // helper 
    bool goal_reached_ = false;
    bool printed_waiting_for_odom_ = false;
    bool printed_waiting_for_traj_ = false;

    bool has_goal_ = false;
    double min_goal_dist_ = 0.75;    

    bool planned_once_ = false;
    bool have_last_time_ = false;

    // State from odom
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool state_received_ = false;

    double robot_radius_ = 0.22;
    double inflation_ = 0.08; 

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


    // VISION STABLIZER CONFIGS =============================================
    // Accept/spam filtering (vision input)
    double accept_goal_change_dist_  = 0.75;

    // Goal smoothing (EMA)
    bool   have_goal_filt_ = false;
    double goal_filt_x_ = 0.0;
    double goal_filt_y_ = 0.0;
    double goal_ema_alpha_ = 0.2;

    // Replan throttling
    double replan_hz_ = 3.0;
    rclcpp::Time last_replan_time_{0,0,RCL_ROS_TIME};
    bool pending_replan_ = false;
    double pending_goal_x_ = 0.0;
    double pending_goal_y_ = 0.0;

    // arived
    double arrive_dist_ = 0.15;
    double arrive_yaw_deg_ = 10.0;

    double v_cmd_max_ = 0.8;
    double w_cmd_max_ = 1.0;
    bool enable_turn_slowdown_ = true;

    std::string planning_frame_ = "odom";
    std::string base_frame_ = "base_link";
    bool require_tf_for_control_ = false;
    double tf_timeout_sec_ = 0.1;


    double lookahead_front_min_x_ = 0.05; // meters
    double k_yaw_ = 1.0;
    bool enable_front_lookahead_guard_ = true;

    // TRAJECTORY HELPERS =============================
    ::TrajectoryController traj_controller_;
    std::vector<::TrajectoryController::Waypoint> waypoints_;
    rclcpp::Time last_time_;


    // Components =======================
    std::unique_ptr<GridMap> grid_map_; // Occupancy grid
    std::unique_ptr<PlannerServer> planner_; // Planning of D* Lite + smoothing + trajecotry conversion
    std::unique_ptr<Trajectory> current_trajectory_; // Current followed plan 



    // USED FOR PRINTING =================================
    struct RectObs {
      double x;
      double y;
      double w;
      double h;
    };
    std::vector<RectObs> obstacle_rects_;

    // LOAD NAV CONFIGS ========================================================
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


      planning_frame_ = this->get_parameter("planning_frame").as_string();

      RCLCPP_INFO(this->get_logger(),"Loaded nav_path config: %s", nav_path.c_str());
      RCLCPP_INFO(this->get_logger(),"Loaded arena_path config: %s", arena_path.c_str());
      try
      {
        YAML::Node nav_cfg = YAML::LoadFile(nav_path);
        YAML::Node arena_cfg = YAML::LoadFile(arena_path);

        // Arena Config =======================================================
        if (arena_cfg["grid"]["width"]) width = arena_cfg["grid"]["width"].as<int>();
        if (arena_cfg["grid"]["height"]) height = arena_cfg["grid"]["height"].as<int>();
        if (arena_cfg["grid"]["resolution"]) resolution_ = arena_cfg["grid"]["resolution"].as<double>();
        if (arena_cfg["origin"]["x"]) origin_.first = arena_cfg["origin"]["x"].as<double>();
        if (arena_cfg["origin"]["y"]) origin_.second = arena_cfg["origin"]["y"].as<double>();

        // Goal =======================================================
        if (arena_cfg["goal"]["x"]) goal_x_ = arena_cfg["goal"]["x"].as<double>();
        if (arena_cfg["goal"]["y"]) goal_y_ = arena_cfg["goal"]["y"].as<double>();

        // Start =======================================================
        if (arena_cfg["start"]["x"]) start_x_ = arena_cfg["start"]["x"].as<double>();
        if (arena_cfg["start"]["y"]) start_y_ = arena_cfg["start"]["y"].as<double>();

        // Nav Config =======================================================
        if (nav_cfg["trajectory"]["speed"]) speed_ = nav_cfg["trajectory"]["speed"].as<double>(); // add speed
        if (nav_cfg["trajectory"]["lookahead_distance"]) lookahead = nav_cfg["trajectory"]["lookahead_distance"].as<double>(); // add lookahead distance
        if (nav_cfg["smoothing"]["max_skip"]) max_skip_ = nav_cfg["smoothing"]["max_skip"].as<int>(); // add max skip


        // PURE PURSUIT =======================================================
        ::TrajectoryController::Config tc{};
        tc.lookahead_dist = static_cast<float>(lookahead);
        tc.cruise_v       = static_cast<float>(speed_);

        tc.max_v = std::max(tc.cruise_v, max_v_);
        tc.max_w = max_w_;
        tc.slowdown_dist = slowdown_dist_;
        tc.pos_tol = pos_tol_;
        tc.advance_tol = advance_tol_;
        tc.preserve_curvature_on_w_saturation = preserve_curvature_on_w_saturation_;
        tc.min_v_near_goal = min_v_near_goal_;
        
        if (nav_cfg["pure_pursuit"]["max_v"]) tc.max_v = std::max(tc.cruise_v, nav_cfg["pure_pursuit"]["max_v"].as<float>()); // add speed
        if (nav_cfg["pure_pursuit"]["max_w"]) tc.max_w = nav_cfg["pure_pursuit"]["max_w"].as<float>(); // add lookahead distance
        if (nav_cfg["pure_pursuit"]["slowdown_dist"]) tc.slowdown_dist = nav_cfg["pure_pursuit"]["slowdown_dist"].as<float>(); // add max skip
        if (nav_cfg["pure_pursuit"]["pos_tol"]) tc.pos_tol = nav_cfg["pure_pursuit"]["pos_tol"].as<float>(); // add speed
        if (nav_cfg["pure_pursuit"]["advance_tol"]) tc.advance_tol = nav_cfg["pure_pursuit"]["advance_tol"].as<float>(); // add lookahead distance
        if (nav_cfg["pure_pursuit"]["preserve_curvature_on_w_saturation"]) tc.preserve_curvature_on_w_saturation = nav_cfg["pure_pursuit"]["preserve_curvature_on_w_saturation"].as<bool>(); // add max skip
        if (nav_cfg["pure_pursuit"]["min_v_near_goal"]) tc.min_v_near_goal = nav_cfg["pure_pursuit"]["min_v_near_goal"].as<float>(); // add max skip
        tc.cruise_v = std::min(tc.cruise_v, tc.max_v);
        traj_controller_.configure(tc);

        // LOADED GRID ============================================
        std::vector<std::vector<int>> grid_data(height, std::vector<int>(width, 0));
        grid_map_ = std::make_unique<GridMap>(grid_data);

        // LOADED Obstacles ============================================
        const YAML::Node &obstacles = arena_cfg["obstacles"];
        for (std::size_t i = 0; i < obstacles.size(); ++i)
        {
          const YAML::Node &obs = obstacles[i];
          if (obs["type"].as<std::string>() == "rectangle")
          {
            mark_rectangle_obstacle(obs);
          }
        }


        RCLCPP_INFO(this->get_logger(), "Configs loaded successfully");
        RCLCPP_INFO(this->get_logger(), "--------------------");
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Config Load Error: %s", e.what());
      }
    }

    // HELPERS ==============================================================
    // SIMIPLIFY GRID PATH, LEAVING CORNERS -----------------
    void compress_collinear(std::vector<::TrajectoryController::Waypoint>& wps, float eps = 1e-4f)
    {
      // Fewer than 3 nothing to compress
      if (wps.size() < 3) return;

      std::vector<::TrajectoryController::Waypoint> out;
      out.reserve(wps.size());
      
      // Kepp first point 
      out.push_back(wps.front());


      // compute for middle points
      for (size_t i = 1; i + 1 < wps.size(); ++i) {
        const auto& p0 = out.back();
        const auto& p1 = wps[i];
        const auto& p2 = wps[i + 1];

        const float ax = p1.x - p0.x;
        const float ay = p1.y - p0.y;
        const float bx = p2.x - p1.x;
        const float by = p2.y - p1.y;

        // cross = detect non colinear turn | dot = make sure same direction
        // if 3 points pass both tests >>> skip p1
        if (std::abs(ax * by - ay * bx) < eps && (ax * bx + ay * by) > 0.0f) continue;

        // keep last
        out.push_back(p1);
      }

      out.push_back(wps.back());
      wps.swap(out);
    }


    // safety for TF failure ------------------ 
    void stop_robot()
    {
      geometry_msgs::msg::Twist z;
      z.linear.x = 0.0;
      z.angular.z = 0.0;
      publisher_->publish(z);
    }

    void goal_reached()
    {
      if (!goal_reached_) {
        goal_reached_ = true;
        std_msgs::msg::Bool reached;
        reached.data = true;
        goal_reached_pub_->publish(reached); // if goal is reached -> publish
      }
    }


    // CONVERTERS ==============================================

    // check if r and c in bounds ----------
    bool cell_in_bounds(int r, int c) const { return grid_map_ && r >= 0 && r < grid_map_->get_rows() && c >= 0 && c < grid_map_->get_cols(); }
    
    // 0.5 >= x >>>> 1 | 0.5 < x >>>> 0
    static inline int rounding_rules(double x) { return (x >= 0.0) ? static_cast<int>(std::floor(x + 0.5)): static_cast<int>(std::ceil (x - 0.5)); }

    // convert from meters >>>> grid coords --------------
    std::pair<int,int> world_to_cell(double x, double y) const { return {rounding_rules((y - origin_.second) / resolution_), rounding_rules((x - origin_.first) / resolution_)}; }

    // converts coords >>> meters --------------
    std::pair<double,double> cell_to_world(int r, int c) const {return {origin_.first + (c + 0.5) * resolution_, origin_.second + (r + 0.5) * resolution_}; }

    // Normalize angle [-pi, pi]
    static double oscolate_around_pi(double a)
    {
      while (a > M_PI) a -= 2.0 * M_PI;
      while (a < -M_PI) a += 2.0 * M_PI;
      return a;
    }

    // convert degrees -> radians
    static double degrees_to_radians(double d) { return d * M_PI / 180.0; }


    // PLAN ======================================================
    // CONFIGURE PLAN / SET GOAL -------------------
    void init_planner()
    {
      // if grid doesnt exist
      if (!grid_map_) {
        RCLCPP_FATAL(this->get_logger(), "grid_map_ non existent. mostlikely configs");
        throw std::runtime_error("INITIALZIE grid_map_");
      }
      // creates plan with params
      PlannerConfig p_cfg;
      p_cfg.speed = speed_;
      p_cfg.max_skip = max_skip_;
      planner_ = std::make_unique<PlannerServer>(*grid_map_, p_cfg);

      auto goal_cell = world_to_cell(goal_x_, goal_y_);

      // check if goal is in bounds
      if (!cell_in_bounds(goal_cell.first, goal_cell.second)) {RCLCPP_ERROR(this->get_logger(), "GOAL OUT OF BOUNDS: world=(%.2f,%.2f) cell=(%d,%d) grid=(%d,%d) origin=(%.2f,%.2f) res=%.2f", goal_x_, goal_y_, goal_cell.first, goal_cell.second, grid_map_->get_rows(), grid_map_->get_cols(), origin_.first, origin_.second, resolution_);}

      // add goal to plan
      planner_->set_goal(GridMap::Cell{goal_cell.first, goal_cell.second});
    }

    // COMPUTE PATH FROM ROBOT POSITION -------------------------
    void compute_plan()
    {
      RCLCPP_INFO(this->get_logger(), "--------------------");
      RCLCPP_INFO(this->get_logger(), "Computing initial path");

      // starting cell
      auto start_cell = world_to_cell(robot_x_, robot_y_);
      if (!cell_in_bounds(start_cell.first, start_cell.second)) {RCLCPP_ERROR(this->get_logger(), "START OUT OF BOUNDS: world=(%.2f,%.2f) cell=(%d,%d)", start_x_, start_y_, start_cell.first, start_cell.second); return;}

      // created plan
      auto trajectory_plan = planner_->compute_plan(GridMap::Cell{start_cell.first, start_cell.second});


      if (! trajectory_plan) {RCLCPP_ERROR(this->get_logger(), "Failed to compute initial path");}

      current_trajectory_ = std::make_unique<Trajectory>();


      // convert each cell -> add point to trajectory
      for (const auto& cell : *trajectory_plan) {
        auto [wx, wy] = cell_to_world(cell.x, cell.y);
        current_trajectory_->add_point(wx, wy, speed_);
      }
      waypoints_.clear();


      // Build waypoints
      for (const auto& p : *current_trajectory_) {
        ::TrajectoryController::Waypoint wp{};
        wp.x = static_cast<float>(p.x);
        wp.y = static_cast<float>(p.y);
        waypoints_.push_back(wp);
      }

      // remove unnecessary waypoints
      compress_collinear(waypoints_);

      // load waypoints
      if (waypoints_.empty()) {
        traj_controller_.clearTrajectory();
      } else {
        traj_controller_.setTrajectory(waypoints_.data(), waypoints_.size());
      }

      // reset so dt not old
      have_last_time_ = false;


      this->publish_and_print_path(*current_trajectory_); // (TO REMOVE): CAN REMOVE LATER JUST VISUALIZES PATH
      print_grid_with_path(*current_trajectory_);  // (TO REMOVE): CAN REMOVE LATER JUST VISUALIZES PATH

    }

    // CLALBACKS =============================================================
    // ODOM TO UPDATE ROBOT POSE  ---------------
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

    }


    // Dynamic goal callback from /goal_pose ----------------------
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      if (!state_received_) {RCLCPP_WARN(this->get_logger(), "Ignoring /goal_pose: no /odom yet"); return;}
      geometry_msgs::msg::PoseStamped goal_ps = *msg;


      // If frame is empty, assume it's already in planning_frame_
      if (goal_ps.header.frame_id.empty()) {goal_ps.header.frame_id = planning_frame_;}

      // Transform into planning frame if needed
      if (goal_ps.header.frame_id != planning_frame_) {
        try {
          goal_ps = tf_buffer_->transform(
            goal_ps,
            planning_frame_,
            tf2::durationFromSec(tf_timeout_sec_)
          );
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "Failed to transform goal from '%s' to '%s': %s", msg->header.frame_id.c_str(), planning_frame_.c_str(), e.what());
          stop_robot();
          return;
        }
      }

      // get the new robot GOAL
      auto goal_raw = std::pair<double,double> {goal_ps.pose.position.x, goal_ps.pose.position.y};
      auto goal_cell = world_to_cell(goal_raw.first, goal_raw.second);
      auto goal_meters = cell_to_world(goal_cell.first, goal_cell.second);


      // check how close robot is
      const double d_robot = std::hypot(goal_meters.first - robot_x_, goal_meters.second - robot_y_);
      if (d_robot < min_goal_dist_) {RCLCPP_WARN(this->get_logger(), "Ignoring /goal_pose: too close (d=%.2f < %.2f). goal=(%.2f,%.2f) robot=(%.2f,%.2f)", d_robot, min_goal_dist_, goal_meters.first, goal_meters.second, robot_x_, robot_y_); return;}
      // Initialize filter if first goal
      if (!have_goal_filt_) {
        goal_filt_x_ = goal_meters.first;
        goal_filt_y_ = goal_meters.second;
        have_goal_filt_ = true;
      } else {
        // Only change if goal changed meaningfully
        const double d_change = std::hypot(goal_meters.first - goal_filt_x_, goal_meters.second - goal_filt_y_);
        if (d_change < accept_goal_change_dist_) {
          // update EMA
          goal_filt_x_ = goal_ema_alpha_ * goal_meters.first + (1.0 - goal_ema_alpha_) * goal_filt_x_;
          goal_filt_y_ = goal_ema_alpha_ * goal_meters.second + (1.0 - goal_ema_alpha_) * goal_filt_y_;
          return;
        }

        // EMA smoothing
        goal_filt_x_ = goal_ema_alpha_ * goal_meters.first + (1.0 - goal_ema_alpha_) * goal_filt_x_;
        goal_filt_y_ = goal_ema_alpha_ * goal_meters.second + (1.0 - goal_ema_alpha_) * goal_filt_y_;
      }

      // Mark a replan
      pending_goal_x_ = goal_filt_x_;
      pending_goal_y_ = goal_filt_y_;
      pending_replan_ = true;

      RCLCPP_INFO(this->get_logger(), "NEW GOAL: (%.2f, %.2f)", pending_goal_x_, pending_goal_y_);
      init_planner();
      compute_plan();
     
      
    }


    // PRINTS TF --------------------------------------
    void print_tf_pose()
    {
      try {
        // Try common frame pairs. One of these will match your sim.
        const std::string parent = "odom";
        const std::string child  = "base_link";

        auto tf = tf_buffer_->lookupTransform(parent, child, tf2::TimePointZero);

        auto tf_coords = std::pair<double, double>{tf.transform.translation.x, tf.transform.translation.y};

        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        RCLCPP_INFO_THROTTLE( this->get_logger(), *this->get_clock(), 250, "TF pose (%s->%s): x=%.3f y=%.3f yaw=%.3f", parent.c_str(), child.c_str(), tf_coords.first, tf_coords.second, yaw);
      } catch (const std::exception &e) {
        RCLCPP_WARN_THROTTLE( this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", e.what());
      }
    }

    
    // Control loop ===============================================
    void control_loop()
    {
      // runs at 10HZ
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "control_loop running");


      // NO ODOM NO TRAJECTORY -> STOP
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

      // // 
      // if (require_tf_for_control_) {
      //   if (!tf_buffer_->canTransform(planning_frame_, base_frame_, tf2::TimePointZero,
      //                                 tf2::durationFromSec(tf_timeout_sec_))) {
      //     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF missing %s->%s, stopping", planning_frame_.c_str(), base_frame_.c_str());
      //     stop_robot();
      //     return;
      //   }
      // }
      
      // no need to check again
      printed_waiting_for_odom_ = false;
      printed_waiting_for_traj_ = false;

      // replan throttle ++++++++++++++++++++++++++++++++++++
      if (pending_replan_) {
        const auto now = this->now();
        // IF ENOUGH TIME HAS PASSED FROM PREVIOUS GOAL UPDATE
        if ((now - last_replan_time_).seconds() >= (1.0 / std::max(0.1, replan_hz_))) {
          // updated goal
          goal_x_ = pending_goal_x_;
          goal_y_ = pending_goal_y_;
          has_goal_ = true;
          goal_reached_ = false;  

          init_planner();
          compute_plan();

          last_replan_time_ = now;
          pending_replan_ = false;

        }
      }

      // dt computation ++++++++++++++++++++++++
      auto now = this->now();
      float dt = 0.1f; // fallback
      // compute dt from last loop 
      if (have_last_time_) dt = static_cast<float>((now - last_time_).seconds());
      last_time_ = now;
      have_last_time_ = true;

      // RAFEED PURE PURSUIT +++++++++++++++++++++++++++
      ::TrajectoryController::Pose2D pose{};
      pose.x = static_cast<float>(robot_x_);
      pose.y = static_cast<float>(robot_y_);
      pose.theta = static_cast<float>(robot_theta_);

      // if dt is bad or too big -> stop
      if (!std::isfinite(dt) || dt <= 0.0f || dt > 0.25f) {
        publisher_->publish(geometry_msgs::msg::Twist()); // STOP
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,"Bad dt=%.3f, stopping", dt);
        return;
      }


      // CALL TRAJECTORY +++++++++++++++++++++++++++
      ::TrajectoryController::Command cmd = traj_controller_.update(pose, dt);


      // CHECK TRAJECOTRY STATUS ++++++++++++++++++++++++
      if (cmd.finished){
        publisher_->publish(geometry_msgs::msg::Twist());
        // if goal is reached
        goal_reached();
        return;
      }


      // CHECK IF GOAL HAS BEEN UPDATED ++++++++++++++++++++++++++
      if (has_goal_) {
        const double d_goal = std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_);
        const double desired_bearing = std::atan2(goal_y_ - robot_y_, goal_x_ - robot_x_);
        const double yaw_err = oscolate_around_pi(desired_bearing - robot_theta_);

        // CHECK ARRIVE DISTANCE && YAW
        if (d_goal < arrive_dist_ && std::abs(yaw_err) < degrees_to_radians(arrive_yaw_deg_)) {
          stop_robot(); // stop if reached
          goal_reached();
          return;
        }
      }

      // SETUP v && w ++++++++++++++++++++++
      double v = cmd.v;
      double w = cmd.w;

      // CLAMP ++++++++++++++++++
      w = std::clamp(w, -w_cmd_max_, w_cmd_max_); // clamp to [-w_max, +w_max]
      v = std::clamp(v, 0.0, v_cmd_max_); // clamp to [0, v_max]

      // SLOW ON TURNS ++++++++++++++++++++++++++++++
      if (enable_turn_slowdown_) {
        const double turn_ratio = std::min(1.0, std::abs(w) / w_cmd_max_);
        const double v_scaled = v_cmd_max_ * (1.0 - 0.7 * turn_ratio);
        v = std::min(v, v_scaled);
        v = std::max(v, 0.0);
      }

      // Front lookahead gaurd +++++++++++++++++++++++++++
      if (enable_front_lookahead_guard_) {
        // compute lookahead
        const double dx = static_cast<double>(cmd.lookahead_x) - robot_x_;
        const double dy = static_cast<double>(cmd.lookahead_y) - robot_y_;

        // find where robot is facing
        const double ct = std::cos(robot_theta_);
        const double st = std::sin(robot_theta_);
        const double x_bl =  ct * dx + st * dy;
        const double y_bl = -st * dx + ct * dy;


        // if behind robot pretty much turn around
        if (x_bl < lookahead_front_min_x_) {
          v = 0.0;
          const double yaw_err = std::atan2(y_bl, x_bl);
          w = std::clamp(k_yaw_ * yaw_err, -w_cmd_max_, w_cmd_max_);
        }
      }

      // PUBLISH to /cmd_vel ++++++++++++++++++++++++++++
      geometry_msgs::msg::Twist out;
      out.linear.x = v;
      out.angular.z = w;
      publisher_->publish(out);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "traj_cmd v=%.3f w=%.3f lookahead=(%.2f,%.2f)",cmd.v, cmd.w, cmd.lookahead_x, cmd.lookahead_y);

    }






    // PRINT THE INFO ===============================================
    // THIS PART IS ONLY FOR DEBUGGING AND FIGURING OUT UNDERLYING ISSUES. THIS HAS NO REAL VALUE BUT TO JUST DEBUG. CAN BE REMOVED LATER IN THE FINAL UPLOAD
    // THIS PART WAS GENERATED TO MAKE MY LIFE EASIER. THIS WILL BE REMOVED LATER
    // PURL FOR PRINTING THE PATH ====================
    nav_msgs::msg::Path to_path_msg(const Trajectory &traj)
    {
      nav_msgs::msg::Path path;

      // Use the node clock (respects use_sim_time)
      path.header.stamp = this->get_clock()->now();

      // Keep frame consistent with what you're already using elsewhere.
      path.header.frame_id = planning_frame_;

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

    // PURLY FOR PRINTING THE PATH ============================
    void publish_and_print_path(const Trajectory &traj)
    {
      // 1) Publish for RViz
      auto path_msg = to_path_msg(traj);
      path_pub_->publish(path_msg);

      // 2) Print a compact summary + first/last points
      const auto &pts = traj.get_points();
      if (pts.empty()){RCLCPP_WARN(this->get_logger(), "=== GENERATED PATH: EMPTY ===");return;}

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


    // PURLY FOR PRINTING OBSTACLES =========================================
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


    // PURELY FOR PRINTING THE GRID ====================================
    void print_grid_with_path(const Trajectory& traj)
    {
      if (!grid_map_){RCLCPP_WARN(this->get_logger(), "print_grid_with_path: grid_map_ is null"); return;}

      const int rows = grid_map_->get_rows();
      const int cols = grid_map_->get_cols();

      // Pack (r,c) -> single key for hash set
      auto pack = [cols](int r, int c) -> long long {return static_cast<long long>(r) * static_cast<long long>(cols) + c;};

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

    
  };

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot_navigation::PathingNode>());
  rclcpp::shutdown();
  return 0;
}
