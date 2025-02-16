#ifndef NAV2_Tracked_MPC_HPP_
#define NAV2_Tracked_MPC_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <gsl/gsl_spline.h>
#include <Eigen/Dense>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "MPC_diffDrive_fblin.h"


namespace tong_controller
{

class TrackedMPC : public nav2_core::Controller
{
public:
  TrackedMPC() = default;
  ~TrackedMPC() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * /*goal_checker*/) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  // Callback executed when a parameter change is detected
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  void interpolateTrajectory(double t, double& x, double& y, double& yaw);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("TrackedMPC")};
  rclcpp::Clock::SharedPtr clock_;

  // Controller configuration parameters
  int prediction_horizon_;
  double q_, r_, p_dist_;
  int max_infeasible_sol_;
  int obstacle_threshold_;
  double MPC_frequency_, fblin_frequency_;
  double w_min_, w_max_, acc_lin_max_, wheel_radius_, track_width_;
  int obstacle_avoidance_range;
  double virtual_laser_interval, virtual_laser_range, search_step;
  int max_search_iterations; 
  double sl_p, sl_a, l_slack, acc_slack;
  

  // Controller internal parameters
  int MPC_execution_counter_;
  double max_linear_velocity, max_angular_velocity;

  MPC_diffDrive_fblin* MPCcontroller;

  geometry_msgs::msg::PoseStamped goal_pose;
  double goal_tolerance_, kp_rot_;

  double path_duration_, path_length_;
  gsl_spline *cspline_x = nullptr; 
  gsl_spline *cspline_y = nullptr;
  gsl_spline *cspline_yaw = nullptr;  
  gsl_interp_accel *acc_x = nullptr; 
  gsl_interp_accel *acc_y = nullptr; 
  gsl_interp_accel *acc_yaw = nullptr; 

  std::vector<double> path_x_, path_y_, path_yaw_;
  long unsigned int path_time_;

  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>  collision_checker_;

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // Publishers for debugging purpose
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> reference_path_pub_;
};

}  

#endif  
