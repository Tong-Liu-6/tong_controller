#ifndef NAV2_FOLLOW_TRAJECTORY_HPP_
#define NAV2_FOLLOW_TRAJECTORY_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
// #include "interpolation.h"

namespace tong_controller
{

class FollowTrajectory : public nav2_core::Controller
{
public:
  FollowTrajectory() = default;
  ~FollowTrajectory() override = default;

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                           const geometry_msgs::msg::Twist& velocity,
                                                           nav2_core::GoalChecker* /*goal_checker*/) override;

  void setPlan(const nav_msgs::msg::Path& path) override;

  void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

  double limitSpeed(const double& expected_angular_speed, const double& max_angular_speed, const double& max_linear_speed);
  nav_msgs::msg::Path path2trajectory(nav_msgs::msg::Path global_path);
  // void getPArray(const nav_msgs::msg::Path& global_trajectory, alglib::real_1d_array& P_time, alglib::real_1d_array& P_x, alglib::real_1d_array& P_y);

private:

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;
  rclcpp::Logger logger_{ rclcpp::get_logger("FollowTrajectory") };
  rclcpp::Clock::SharedPtr clock_;
  nav_msgs::msg::Path global_plan_;
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>> collision_checker_;

  // Controller params
  std::string base_frame_, map_frame_;
  double max_vel_linear_, max_vel_angular_, preview_length_; 
  nav_msgs::msg::Path global_trajectory;
  // alglib::spline1dinterpolant cubicspline_x, cubicspline_y;




  double kp_, kp_rot_, ki_, lookahead_, goal_tolerance_; 
  
  double dt, integrator_x, integrator_y;
  double v, omega;
  double linear_vel, angular_vel;

  // Dynamic parameters handler
  std::mutex mutex_;
};

}  // namespace tong_controller

#endif