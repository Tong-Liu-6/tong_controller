#include <nav2_core/exceptions.hpp>
#include "tong_controller.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace tong_controller
{
void TrackedMPC::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                           std::shared_ptr<tf2_ros::Buffer> tf,
                           std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node)
  {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // params
  declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".map_frame", rclcpp::ParameterValue("map"));
  node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  node->get_parameter(plugin_name_ + ".map_frame", map_frame_);

  declare_parameter_if_not_declared(node, plugin_name_ + ".angular_speed", rclcpp::ParameterValue(0.5));
  node->get_parameter(plugin_name_ + ".angular_speed", angular_speed);

  double controller_freqency;
  node->get_parameter("controller_frequency", controller_freqency);

  RCLCPP_INFO(logger_, "TrackedMPC initialized!");

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void TrackedMPC::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type "
              "tong_controller::TrackedMPC",
              plugin_name_.c_str());
}

void TrackedMPC::activate()
{
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "tong_controller::TrackedMPC",
              plugin_name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
}

void TrackedMPC::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "tong_controller::TrackedMPC",
              plugin_name_.c_str());
}

void TrackedMPC::setPlan(const nav_msgs::msg::Path& path)
{
  RCLCPP_INFO(logger_, "Got new plan");
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped TrackedMPC::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                        const geometry_msgs::msg::Twist& speed,
                                                                        nav2_core::GoalChecker* goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
  {
   RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  }

  // get speed
  double vt = std::hypot(speed.linear.x, speed.linear.y);
  double wt = speed.angular.z;

  // publish command
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.angular.z = angular_speed;

  return cmd_vel;
}

void TrackedMPC::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  // 
}

}  // namespace tong_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tong_controller::TrackedMPC, nav2_core::Controller)