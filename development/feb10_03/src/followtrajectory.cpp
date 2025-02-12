#include <nav2_core/exceptions.hpp>
#include "followtrajectory.hpp"

using nav2_util::declare_parameter_if_not_declared;
namespace tong_controller
{
void FollowTrajectory::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
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
  // frame name
  declare_parameter_if_not_declared(node, plugin_name_ + ".base_frame", rclcpp::ParameterValue("base_link"));
  declare_parameter_if_not_declared(node, plugin_name_ + ".map_frame", rclcpp::ParameterValue("map"));
  node->get_parameter(plugin_name_ + ".base_frame", base_frame_);
  node->get_parameter(plugin_name_ + ".map_frame", map_frame_);

  // speed limitation
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_linear", rclcpp::ParameterValue(0.2));
  node->get_parameter(plugin_name_ + ".max_vel_linear", max_vel_linear_);
  declare_parameter_if_not_declared(node, plugin_name_ + ".max_vel_angular", rclcpp::ParameterValue(2.0));
  node->get_parameter(plugin_name_ + ".max_vel_angular", max_vel_angular_);

  // params
  declare_parameter_if_not_declared(node, plugin_name_ + ".preview_length", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".preview_length", preview_length_);

  // PI controller
  declare_parameter_if_not_declared(node, plugin_name_ + ".kp", rclcpp::ParameterValue(1.5));
  node->get_parameter(plugin_name_ + ".kp", kp_);
  declare_parameter_if_not_declared(node, plugin_name_ + ".ki", rclcpp::ParameterValue(0.005));
  node->get_parameter(plugin_name_ + ".ki", ki_);
  declare_parameter_if_not_declared(node, plugin_name_ + ".pi_freq_ratio", rclcpp::ParameterValue(5));
  node->get_parameter(plugin_name_ + ".pi_freq_ratio", pi_freq_ratio_);


  
  // declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead", rclcpp::ParameterValue(0.25));
  // node->get_parameter(plugin_name_ + ".lookahead", lookahead_);
  // goal orientation (P controller)
  // declare_parameter_if_not_declared(node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.1));
  // node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);
  // declare_parameter_if_not_declared(node, plugin_name_ + ".kp_rot", rclcpp::ParameterValue(1.0));
  // node->get_parameter(plugin_name_ + ".kp_rot", kp_rot_);

  double controller_freqency;
  node->get_parameter("controller_frequency", controller_freqency);

  RCLCPP_INFO(logger_, "FollowTrajectory initialized!");

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>>(costmap_);
  collision_checker_->setCostmap(costmap_);
}

void FollowTrajectory::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type "
              "tong_controller::FollowTrajectory",
              plugin_name_.c_str());
}

void FollowTrajectory::activate()
{
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "tong_controller::FollowTrajectory",
              plugin_name_.c_str());
  // Add callback for dynamic parameters
  auto node = node_.lock();
}

void FollowTrajectory::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "tong_controller::FollowTrajectory",
              plugin_name_.c_str());
}

void FollowTrajectory::setPlan(const nav_msgs::msg::Path& path)
{
  RCLCPP_INFO(logger_, "Got new plan");
  global_plan_ = path;
  global_trajectory = path2trajectory(global_plan_);

  std::vector<double> P_time, P_x, P_y;
  getPArray(global_trajectory, P_time, P_x, P_y);
  goal_time = P_time.back();

  size_t n = P_time.size();
  if (cspline_x) {
    gsl_spline_free(cspline_x);
  }
  if (cspline_y) {
    gsl_spline_free(cspline_y);
  }
  if (acc_x) {
    gsl_interp_accel_free(acc_x);
  }
  if (acc_y) {
    gsl_interp_accel_free(acc_y);
  }
  cspline_x = gsl_spline_alloc(gsl_interp_cspline, n);
  cspline_y = gsl_spline_alloc(gsl_interp_cspline, n);
  acc_x = gsl_interp_accel_alloc();
  acc_y = gsl_interp_accel_alloc();
  gsl_spline_init(cspline_x, P_time.data(), P_x.data(), n);
  gsl_spline_init(cspline_y, P_time.data(), P_y.data(), n);

  // double a = gsl_spline_eval(cspline_x, 10.2, acc_x);
  // double b = gsl_spline_eval_deriv(cspline_x, 10.2, acc_x);
  // RCLCPP_INFO(logger_, "!!!!!!!!!!TEST: a = %.2f, b = %.2f", a, b);

  // integrator initialization
  integrator_x = 0.0;
  integrator_y = 0.0;
  loop_index = 0;
  rclcpp::Time now = clock_->now();
  start_time = now.seconds();

}

geometry_msgs::msg::TwistStamped FollowTrajectory::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
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

  // get info
  double vt = std::hypot(speed.linear.x, speed.linear.y);
  double wt = speed.angular.z;
  double theta = tf2::getYaw(pose.pose.orientation);
  rclcpp::Time now = clock_->now();
  double current_time = now.seconds() - start_time;
  if (current_time > goal_time) current_time = goal_time;

  // edit command message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  // control orientation at goal
  // geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses[global_plan_.poses.size() - 1];
  // double goal_dist = nav2_util::geometry_utils::euclidean_distance(pose.pose.position, goal_pose.pose.position);
  // if (goal_dist < goal_tolerance_) {
  //   cmd_vel.twist.linear.x = 0.0;
  //   double d_theta = tf2::getYaw(goal_pose.pose.orientation) - theta;
  //   while (d_theta > M_PI) d_theta -= 2 * M_PI; 
  //   while (d_theta < -M_PI) d_theta += 2 * M_PI; 
  //   angular_vel = kp_rot_ * d_theta;
  //   cmd_vel.twist.angular.z = std::max(-max_vel_angular_, std::min(angular_vel, max_vel_angular_));
  //   return cmd_vel;
  // }

  // feedback linearization
  // calculate point P
  geometry_msgs::msg::PoseStamped preview_point; 
  preview_point.pose.position.x = pose.pose.position.x + preview_length_ * cos(theta); 
  preview_point.pose.position.y = pose.pose.position.y + preview_length_ * sin(theta);

  // FF + PI controller
  if (loop_index % pi_freq_ratio_ == 0) {
    double desire_x = gsl_spline_eval(cspline_x, current_time, acc_x);
    double desire_y = gsl_spline_eval(cspline_y, current_time, acc_y);
    double desire_xd = gsl_spline_eval_deriv(cspline_x, current_time, acc_x);
    double desire_yd = gsl_spline_eval_deriv(cspline_y, current_time, acc_y);

    double dx = desire_x - preview_point.pose.position.x;
    double dy = desire_y - preview_point.pose.position.y;
    integrator_x += dx;
    integrator_y += dy;

    v_xp = desire_xd + kp_ * dx + ki_ * integrator_x; 
    v_yp = desire_yd + kp_ * dy + ki_ * integrator_y;

  }

  // kinematic model linearization
  linear_vel = v_xp * cos(theta) + v_yp * sin(theta);
  angular_vel = (v_yp * cos(theta) - v_xp * sin(theta)) / preview_length_;

  // limit speed
  cmd_vel.twist.angular.z = std::max(-max_vel_angular_, std::min(angular_vel, max_vel_angular_));
  double linear_limit = limitSpeed(cmd_vel.twist.angular.z, max_vel_angular_, max_vel_linear_);
  cmd_vel.twist.linear.x = std::max(-linear_limit, std::min(linear_vel, linear_limit));

  loop_index++;
  return cmd_vel;
}

void FollowTrajectory::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  // 
}

double FollowTrajectory::limitSpeed(const double& expected_angular_speed, const double& max_angular_speed, const double& max_linear_speed)
{
  double limit_linear_speed = max_linear_speed * (1 - std::abs(expected_angular_speed) / max_angular_speed);
  return limit_linear_speed;
}

nav_msgs::msg::Path FollowTrajectory::path2trajectory(nav_msgs::msg::Path global_path)
{ 
  size_t path_length = global_path.poses.size();

  double point_time = 0.0;
  int32_t point_time_s = static_cast<int32_t>(point_time);
  uint32_t point_time_ns = static_cast<uint32_t>((point_time - point_time_s) * 1e9);
  global_path.poses[0].header.stamp.sec = point_time_s;
  global_path.poses[0].header.stamp.nanosec = point_time_ns;

  global_path.poses[0].pose.orientation.z = tf2::getYaw(global_path.poses[0].pose.orientation);
  global_path.poses[path_length - 1].pose.orientation.z = tf2::getYaw(global_path.poses[path_length - 1].pose.orientation);

  double point_yaw, interval_time;

  for (size_t i = 1; i < path_length; i++) {
    if (i != path_length - 1) {
      point_yaw = atan2(global_path.poses[i].pose.position.y - global_path.poses[i - 1].pose.position.y, global_path.poses[i].pose.position.x - global_path.poses[i - 1].pose.position.x);
      global_path.poses[i].pose.orientation.z = point_yaw;
    }
    
    double d_yaw = global_path.poses[i].pose.orientation.z - global_path.poses[i - 1].pose.orientation.z;
    while (d_yaw > M_PI) d_yaw -= 2 * M_PI; 
    while (d_yaw < -M_PI) d_yaw += 2 * M_PI;
    double d_distance = nav2_util::geometry_utils::euclidean_distance(global_path.poses[i].pose.position, global_path.poses[i - 1].pose.position);
    interval_time = d_yaw / max_vel_angular_ + d_distance / max_vel_linear_;
    point_time += interval_time;
    int32_t point_time_s = static_cast<int32_t>(point_time);
    uint32_t point_time_ns = static_cast<uint32_t>((point_time - point_time_s) * 1e9);
    global_path.poses[i].header.stamp.sec = point_time_s;
    global_path.poses[i].header.stamp.nanosec = point_time_ns;
  }

  return global_path;
}

void FollowTrajectory::getPArray(const nav_msgs::msg::Path& global_trajectory, std::vector<double>& P_time, std::vector<double>& P_x, std::vector<double>& P_y)
{
  size_t trajectory_length = global_trajectory.poses.size();
  P_time.clear();
  P_x.clear();
  P_y.clear();
  P_time.resize(trajectory_length);
  P_x.resize(trajectory_length);
  P_y.resize(trajectory_length);
  for (size_t i = 0; i < trajectory_length; i++) {
    P_time[i] = 1.0 * global_trajectory.poses[i].header.stamp.sec + 1e-9 * global_trajectory.poses[i].header.stamp.nanosec;
    P_x[i] = global_trajectory.poses[i].pose.position.x + preview_length_ * cos(global_trajectory.poses[i].pose.orientation.z); 
    P_y[i] = global_trajectory.poses[i].pose.position.y + preview_length_ * sin(global_trajectory.poses[i].pose.orientation.z);
    if (i != 0 && P_time[i]<= P_time[i - 1]) P_time[i] = P_time[i - 1] + 0.01; 
  }
}

}  // namespace tong_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tong_controller::FollowTrajectory, nav2_core::Controller)