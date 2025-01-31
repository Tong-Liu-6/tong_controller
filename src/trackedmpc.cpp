#include <string>
#include <vector>

#include "trackedmpc.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace tong_controller {

    void handleDebugMessages(const std::string& message) {
        RCLCPP_DEBUG(rclcpp::get_logger("TrackedMPC"), "%s", message.c_str());
    }

    void handleInfoMessages(const std::string& message) {
        RCLCPP_INFO(rclcpp::get_logger("TrackedMPC"), "%s", message.c_str());
    }

    void handleErrorMessages(const std::string& message) {
        RCLCPP_INFO(rclcpp::get_logger("TrackedMPC"), "%s", message.c_str());
    }

    void TrackedMPC::configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {

        auto node = parent.lock();
        node_ = parent;
        if (!node) {
            throw nav2_core::PlannerException("Unable to lock node!");
        }

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        // Controller parameters
        declare_parameter_if_not_declared(node, "controller_frequency", rclcpp::ParameterValue(100.0));
        if (false == node->get_parameter("controller_frequency", fblin_frequency_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "controller_frequency");

        declare_parameter_if_not_declared(node, plugin_name_ + ".prediction_horizon", rclcpp::ParameterValue(10));
        if (false == node->get_parameter(plugin_name_ + ".prediction_horizon", prediction_horizon_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "prediction_horizon");

        declare_parameter_if_not_declared(node, plugin_name_ + ".use_tracking_controller", rclcpp::ParameterValue(false));
        if (false == node->get_parameter(plugin_name_ + ".use_tracking_controller", use_tracking_controller_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "use_tracking_controller");

        declare_parameter_if_not_declared(node, plugin_name_ + ".MPC_frequency", rclcpp::ParameterValue(10.0));
        if (false == node->get_parameter(plugin_name_ + ".MPC_frequency", MPC_frequency_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "MPC_frequency");

        declare_parameter_if_not_declared(node, plugin_name_ + ".q", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".q", q_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "q");

        declare_parameter_if_not_declared(node, plugin_name_ + ".r", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".r", r_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "r");

        declare_parameter_if_not_declared(node, plugin_name_ + ".p_dist", rclcpp::ParameterValue(0.25));
        if (false == node->get_parameter(plugin_name_ + ".p_dist", p_dist_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "p_dist");

        declare_parameter_if_not_declared(node, plugin_name_ + ".max_infeasible_sol", rclcpp::ParameterValue(5));
        if (false == node->get_parameter(plugin_name_ + ".max_infeasible_sol", max_infeasible_sol_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "max_infeasible_sol");

        declare_parameter_if_not_declared(node, plugin_name_ + ".w_min", rclcpp::ParameterValue(-0.1));
        if (false == node->get_parameter(plugin_name_ + ".w_min", w_min_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "w_min");

        declare_parameter_if_not_declared(node, plugin_name_ + ".w_max", rclcpp::ParameterValue(0.1));
        if (false == node->get_parameter(plugin_name_ + ".w_max", w_max_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "w_max");

        declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lin_max", rclcpp::ParameterValue(0.1));
        if (false == node->get_parameter(plugin_name_ + ".acc_lin_max", acc_lin_max_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "acc_lin_max");

        declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_radius", rclcpp::ParameterValue(0.1));
        if (false == node->get_parameter(plugin_name_ + ".wheel_radius", wheel_radius_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "wheel_radius");

        declare_parameter_if_not_declared(node, plugin_name_ + ".track_width", rclcpp::ParameterValue(0.5));
        if (false == node->get_parameter(plugin_name_ + ".track_width", track_width_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "track_width");

        declare_parameter_if_not_declared(node, plugin_name_ + ".path_sampling_threshold", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".path_sampling_threshold", path_sampling_threshold_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "path_sampling_threshold");

        declare_parameter_if_not_declared(node, plugin_name_ + ".next_goal_threshold", rclcpp::ParameterValue(0.1));
        if (false == node->get_parameter(plugin_name_ + ".next_goal_threshold", next_goal_threshold_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "next_goal_threshold");

        // Check parameter consistency
        if (std::fmod(fblin_frequency_, MPC_frequency_)!=0.0) {
            RCLCPP_ERROR(logger_, "MPC_frequency must be a multiple of controller_frequency");
        }

        // Create publishers
        next_goal_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("next_goal_point", 1);
        reference_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("reference_path", 1);

        // Initialize collision checker and set costmap
        collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker < nav2_costmap_2d::Costmap2D * >> (costmap_);
        collision_checker_->setCostmap(costmap_);

        // Create and initialize MPC controller
        MPCcontroller = NULL;
        if (!(MPCcontroller = new MPC_diffDrive_fblin())) {
            RCLCPP_ERROR(logger_, "Unable to create MPC controller class");
        }

        MPC_execution_counter_ = 0;

        MPCcontroller->set_ErrorMsgCallback(handleErrorMessages);
        MPCcontroller->set_DebugMsgCallback(handleDebugMessages);
        MPCcontroller->set_InfoMsgCallback(handleInfoMessages);

        MPCcontroller->set_MPCparams(1.0/MPC_frequency_, prediction_horizon_, q_, r_);
        MPCcontroller->set_FBLINparams(1.0/fblin_frequency_, p_dist_);
        MPCcontroller->set_robotParams(w_max_, w_min_, wheel_radius_, track_width_, acc_lin_max_/wheel_radius_);

        if (!MPCcontroller->initialize()) {
            RCLCPP_ERROR(logger_, "Unable to initialize MPC controller");
        }

        path_duration_ = path_length_ = 0.0;
        path_idx_ = path_time_ = 0;
    }

    void TrackedMPC::cleanup() {
        RCLCPP_INFO(logger_, "Cleaning up controller: %s of type tong_controller::TrackedMPC", plugin_name_.c_str());

        // Reset publishers
        next_goal_pub_.reset();
        reference_path_pub_.reset();

        // Delete MPC controller object
        if (MPCcontroller) {
            delete MPCcontroller;
        }
    }

    void TrackedMPC::activate() {
        RCLCPP_INFO(logger_, "Activating controller: %s of type tong_controller::TrackedMPC", plugin_name_.c_str());

        // Activate publishers
        next_goal_pub_->on_activate();
        reference_path_pub_->on_activate();

        // Add callback for dynamic parameters
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(
                std::bind(&TrackedMPC::dynamicParametersCallback, this, std::placeholders::_1));
    }

    void TrackedMPC::deactivate() {
        RCLCPP_INFO(logger_, "Deactivating controller: %s of type tong_controller::TrackedMPC", plugin_name_.c_str());

        // Deactivate publishers
        next_goal_pub_->on_deactivate();
        reference_path_pub_->on_deactivate();

        // Reset dynamic parameter handler
        dyn_params_handler_.reset();
    }

    geometry_msgs::msg::TwistStamped TrackedMPC::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist& /*speed*/, nav2_core::GoalChecker* /*goal_checker*/) {

        std::lock_guard<std::mutex> lock_reinit(mutex_);

        nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

        // Update controller state
        MPCcontroller->set_actualRobotState(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));

        // Execute MPC controller
        // TBD: Add replanning in case of too much MPC failures
        if (MPC_execution_counter_ == (int)fblin_frequency_/MPC_frequency_) {
            // Update MPC reference
            if (use_tracking_controller_) {
                // Check that a plan has been received
                if ((path_duration_==0.0) || (path_length_==0.0)) {
                    RCLCPP_ERROR(logger_, "No plan available, cannot start the controller");

                    // Set the reference equal to the actual pose
                    MPCcontroller->set_referenceRobotState(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
                } else {
                    // Compute the reference robot state along the prediction horizon
                    Eigen::VectorXd referenceRobotState;
                    referenceRobotState.resize(3 * (prediction_horizon_ + 1), 1);
                    for (long unsigned int k = 0; k < (long unsigned int)(prediction_horizon_ + 1); k++) {
                        double x, y, yaw;
                        interpolateTrajectory((path_time_+k)/MPC_frequency_, x, y, yaw);

                        referenceRobotState(3 * k) = x;
                        referenceRobotState(3 * k + 1) = y;
                        referenceRobotState(3 * k + 2) = yaw;
                    }

                    // Update the path time
                    path_time_++;

                    // Update the MPC reference vector
                    MPCcontroller->set_referenceRobotState(referenceRobotState);

                    // Publish reference trajectory
                    nav_msgs::msg::Path reference_path;
                    reference_path.header.frame_id = pose.header.frame_id;
                    reference_path.header.stamp = pose.header.stamp;
                    for (long unsigned int k = 0; k < (long unsigned int) prediction_horizon_ + 1; k++) {
                        geometry_msgs::msg::PoseStamped path_pose;
                        path_pose.header.frame_id = "map";
                        path_pose.header.stamp = pose.header.stamp;
                        path_pose.pose.position.x = referenceRobotState(3*k);
                        path_pose.pose.position.y = referenceRobotState(3 * k + 1);
                        path_pose.pose.position.z = 0.0;
                        tf2::Quaternion tf2_quat;
                        tf2_quat.setRPY(0.0, 0.0, referenceRobotState(3 * k + 2));
                        path_pose.pose.orientation = tf2::toMsg(tf2_quat);

                        reference_path.poses.push_back(path_pose);
                    }

                    reference_path_pub_->publish(reference_path);
                }
            } else {
                // Check that a plan has been received
                if ((path_x_.size()==0) || (path_y_.size()==0) || (path_yaw_.size()==0)) {
                    RCLCPP_ERROR(logger_, "No plan available, cannot start the controller");

                    // Set the reference equal to the actual pose
                    MPCcontroller->set_referenceRobotState(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
                } else {
                    // If I'm close enough to the goal I switch to next one
                    if (std::pow(std::pow(pose.pose.position.x - path_x_.at(path_idx_), 2.0) +
                                 std::pow(pose.pose.position.y - path_y_.at(path_idx_), 2.0), 0.5) < next_goal_threshold_ &&
                        (path_idx_ < (path_x_.size() - 1))) {
                        path_idx_++;
                    }

                    // Set the goal as current reference
                    MPCcontroller->set_referenceRobotState(path_x_.at(path_idx_), path_y_.at(path_idx_),
                                                           path_yaw_.at(path_idx_));

                    // Publish goal pose
                    geometry_msgs::msg::PoseStamped goal_pose;
                    goal_pose.header.frame_id = "map";
                    goal_pose.header.stamp = pose.header.stamp;
                    goal_pose.pose.position.x = path_x_.at(path_idx_);
                    goal_pose.pose.position.y = path_y_.at(path_idx_);
                    goal_pose.pose.position.z = 0.0;
                    tf2::Quaternion tf2_quat;
                    tf2_quat.setRPY(0.0, 0.0, path_yaw_.at(path_idx_));
                    goal_pose.pose.orientation = tf2::toMsg(tf2_quat);

                    next_goal_pub_->publish(goal_pose);
                }
            }

            // Execute MPC controller
            if (!MPCcontroller->executeMPCcontroller())
                RCLCPP_ERROR(logger_, "No optimal solution found");

            MPC_execution_counter_ = 0;
        } else {
            MPC_execution_counter_++;
        }

        // Execute feedback linearization controller
        MPCcontroller->executeLinearizationController();

        // Get actual control signal
        double linear_vel, angular_vel;
        MPCcontroller->get_actualControl(linear_vel, angular_vel);

        // Populate and return message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    void TrackedMPC::setPlan(const nav_msgs::msg::Path &path) {
        if (use_tracking_controller_) {
            // Natural coordinate computation
            std::vector<double> path_x(path.poses.size(), 0.0);
            std::vector<double> path_y(path.poses.size(), 0.0);
            std::vector<double> path_yaw(path.poses.size(), 0.0);
            std::vector<double> path_s(path.poses.size(), 0.0);

            path_x.at(0) = path.poses.at(0).pose.position.x;
            path_y.at(0) = path.poses.at(0).pose.position.y;
            path_yaw.at(0) = tf2::getYaw(path.poses.at(0).pose.orientation);
            for (long unsigned int k=1; k<path.poses.size(); k++) {
                path_s.at(k) = path_s.at(k-1)+
                               std::pow(std::pow(path.poses.at(k).pose.position.x-path.poses.at(k-1).pose.position.x, 2.0)+
                                        std::pow(path.poses.at(k).pose.position.y-path.poses.at(k-1).pose.position.y, 2.0), 0.5);
                path_x.at(k) = path.poses.at(k).pose.position.x;
                path_y.at(k) = path.poses.at(k).pose.position.y;
                path_yaw.at(k) = tf2::getYaw(path.poses.at(k).pose.orientation);

                // s values must be strictly increasing for interpolation
                if (path_s.at(k) <= path_s.at(k - 1)) path_s.at(k) = path_s.at(k - 1) + 0.01;
            }

            // Interpolation of the x, y, yaw components of the path with respect to the natural coordinate using a cubic spline
            // try {
            //     alglib::real_1d_array x, y, yaw, s;
            //     x.setcontent(path_x.size(), &path_x[0]);
            //     y.setcontent(path_y.size(), &path_y[0]);
            //     yaw.setcontent(path_yaw.size(), &path_yaw[0]);
            //     s.setcontent(path_s.size(), &path_s[0]);

            //     spline1dbuildcubic(s, x, path_sp_x_);
            //     spline1dbuildcubic(s, y, path_sp_y_);
            //     spline1dbuildcubic(s, yaw, path_sp_yaw_);
            // } catch(alglib::ap_error alglib_exception)
            // {
            //     RCLCPP_ERROR(logger_, "ALGLIB exception with message '%s'\n", alglib_exception.msg.c_str());
            // }

            size_t cspline_n = path_x.size();
            if (cspline_x) {
                gsl_spline_free(cspline_x);
            }
            if (cspline_y) {
                gsl_spline_free(cspline_y);
            }
            if (cspline_yaw) {
                gsl_spline_free(cspline_yaw);
            }
            if (acc_x) {
                gsl_interp_accel_free(acc_x);
            }
            if (acc_y) {
                gsl_interp_accel_free(acc_y);
            }
            if (acc_yaw) {
                gsl_interp_accel_free(acc_yaw);
            }
            cspline_x = gsl_spline_alloc(gsl_interp_cspline, cspline_n);
            cspline_y = gsl_spline_alloc(gsl_interp_cspline, cspline_n);
            cspline_yaw = gsl_spline_alloc(gsl_interp_cspline, cspline_n);
            acc_x = gsl_interp_accel_alloc();
            acc_y = gsl_interp_accel_alloc();
            acc_yaw = gsl_interp_accel_alloc();
            gsl_spline_init(cspline_x, path_s.data(), path_x.data(), cspline_n);
            gsl_spline_init(cspline_y, path_s.data(), path_y.data(), cspline_n);
            gsl_spline_init(cspline_yaw, path_s.data(), path_yaw.data(), cspline_n);


            // Computation of the path length and duration (according to the maximum velocity and acceleration)
            path_length_ = path_s.back();
            path_duration_ = std::fmax(1.5*path_length_/(w_max_*wheel_radius_), std::pow(6.0*path_length_/acc_lin_max_, 0.5));
            path_time_ = 0;
        } else {
            // Clear all previous elements
            path_x_.clear();
            path_y_.clear();
            path_yaw_.clear();

            // Store the first element of the path
            path_x_.push_back(path.poses.at(1).pose.position.x);
            path_y_.push_back(path.poses.at(1).pose.position.y);
            path_yaw_.push_back(tf2::getYaw(path.poses.at(1).pose.orientation));

            // Sample the path with a distance path_sampling_threshold_
            for (long unsigned int k=2; k<path.poses.size(); k++) {
                if (std::pow(std::pow(path.poses.at(k).pose.position.x-path_x_.back(), 2.0)+
                std::pow(path.poses.at(k).pose.position.y-path_y_.back(), 2.0), 0.5)>=path_sampling_threshold_ ||
                k==path.poses.size()-1) {
                    path_x_.push_back(path.poses.at(k).pose.position.x);
                    path_y_.push_back(path.poses.at(k).pose.position.y);
                    path_yaw_.push_back(tf2::getYaw(path.poses.at(k).pose.orientation));
                }
            }

            // Initialize path index
            path_idx_ = 0;
        }

        MPCcontroller->reset_pre_vP();
        
    }

    void TrackedMPC::setSpeedLimit(const double &speed_limit, const bool &percentage) {
        if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
            // Restore default value
            max_linear_velocity = w_max_*wheel_radius_;
            max_angular_velocity = (w_max_-w_min_)/track_width_*wheel_radius_;
        } else {
            if (percentage) {
                // Speed limit is expressed in % from maximum speed of robot
                max_linear_velocity = w_max_*wheel_radius_ * speed_limit / 100.0;
                max_angular_velocity = (max_linear_velocity/wheel_radius_-w_min_)/track_width_*wheel_radius_;
            } else {
                // Speed limit is expressed in absolute value
                max_linear_velocity = speed_limit;
                max_angular_velocity = (max_linear_velocity/wheel_radius_-w_min_)/track_width_*wheel_radius_;
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult TrackedMPC::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        std::lock_guard<std::mutex> lock_reinit(mutex_);

        for (auto parameter: parameters) {
            const auto &type = parameter.get_type();
            const auto &name = parameter.get_name();

            if (type == ParameterType::PARAMETER_DOUBLE) {
                if (name == plugin_name_ + ".MPC_frequency") {
                    MPC_frequency_ = parameter.as_double();
                } else if (name == "controller_frequency") {
                    fblin_frequency_ = parameter.as_double();
                } else if (name == plugin_name_ + ".q") {
                    q_ = parameter.as_double();
                } else if (name == plugin_name_ + ".r") {
                    r_ = parameter.as_double();
                } else if (name == plugin_name_ + ".p_dist") {
                    p_dist_ = parameter.as_double();
                } else if (name == plugin_name_ + ".w_min") {
                    w_min_ = parameter.as_double();
                } else if (name == plugin_name_ + ".w_max") {
                    w_max_ = parameter.as_double();
                } else if (name == plugin_name_ + ".acc_lin_max") {
                    acc_lin_max_ = parameter.as_double();
                } else if (name == plugin_name_ + ".wheel_radius") {
                    wheel_radius_ = parameter.as_double();
                } else if (name == plugin_name_ + ".track_width") {
                    track_width_ = parameter.as_double();
                } else if (name == plugin_name_ + ".path_sampling_threshold") {
                    path_sampling_threshold_ = parameter.as_double();
                } else if (name == plugin_name_ + ".next_goal_threshold") {
                    next_goal_threshold_ = parameter.as_double();
                }
            } else if (type == ParameterType::PARAMETER_INTEGER) {
                if (name == plugin_name_ + ".prediction_horizon") {
                    prediction_horizon_ = parameter.as_int();
                } else if (name == plugin_name_ + ".max_infeasible_sol") {
                    max_infeasible_sol_ = parameter.as_int();
                }
            } else if (type == ParameterType::PARAMETER_BOOL) {
                if (name == plugin_name_ + "use_tracking_controller") {
                    use_tracking_controller_ = parameter.as_bool();
                }
            }
        }

        result.successful = true;
        return result;
    }

    void TrackedMPC::interpolateTrajectory(double t, double& x, double& y, double& yaw) {
        double s = path_length_*(3.0*std::pow(std::min(t/path_duration_, path_duration_), 2.0)-
                2.0*std::pow(std::min(t/path_duration_, path_duration_), 3.0));

        // x = spline1dcalc(path_sp_x_, s);
        // y = spline1dcalc(path_sp_y_, s);
        // yaw = spline1dcalc(path_sp_yaw_, s);

        x = gsl_spline_eval(cspline_x, s, acc_x);
        y = gsl_spline_eval(cspline_y, s, acc_y);
        yaw = gsl_spline_eval(cspline_yaw, s, acc_yaw);

    }
}  // namespace tong_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(tong_controller::TrackedMPC, nav2_core::Controller)
