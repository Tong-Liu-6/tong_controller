#include <string>
#include <vector>

#include "diffmpc.hpp"
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
        RCLCPP_DEBUG(rclcpp::get_logger("DiffMPC"), "%s", message.c_str());
    }

    void handleInfoMessages(const std::string& message) {
        RCLCPP_INFO(rclcpp::get_logger("DiffMPC"), "%s", message.c_str());
    }

    void handleErrorMessages(const std::string& message) {
        RCLCPP_INFO(rclcpp::get_logger("DiffMPC"), "%s", message.c_str());
    }

    void DiffMPC::configure(
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
        declare_parameter_if_not_declared(node, "controller_frequency", rclcpp::ParameterValue(50.0));
        if (false == node->get_parameter("controller_frequency", fblin_frequency_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "controller_frequency");

        declare_parameter_if_not_declared(node, plugin_name_ + ".prediction_horizon", rclcpp::ParameterValue(10));
        if (false == node->get_parameter(plugin_name_ + ".prediction_horizon", prediction_horizon_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "prediction_horizon");

        declare_parameter_if_not_declared(node, plugin_name_ + ".MPC_frequency", rclcpp::ParameterValue(10.0));
        if (false == node->get_parameter(plugin_name_ + ".MPC_frequency", MPC_frequency_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "MPC_frequency");

        declare_parameter_if_not_declared(node, plugin_name_ + ".q", rclcpp::ParameterValue(3.0));
        if (false == node->get_parameter(plugin_name_ + ".q", q_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "q");

        declare_parameter_if_not_declared(node, plugin_name_ + ".r", rclcpp::ParameterValue(0.3));
        if (false == node->get_parameter(plugin_name_ + ".r", r_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "r");

        declare_parameter_if_not_declared(node, plugin_name_ + ".p_dist", rclcpp::ParameterValue(0.1));
        if (false == node->get_parameter(plugin_name_ + ".p_dist", p_dist_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "p_dist");

        declare_parameter_if_not_declared(node, plugin_name_ + ".max_infeasible_sol", rclcpp::ParameterValue(5));
        if (false == node->get_parameter(plugin_name_ + ".max_infeasible_sol", max_infeasible_sol_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "max_infeasible_sol");

        declare_parameter_if_not_declared(node, plugin_name_ + ".max_ref_delay", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".max_ref_delay", max_ref_delay_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "max_ref_delay");

        declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_threshold", rclcpp::ParameterValue(251));
        if (false == node->get_parameter(plugin_name_ + ".obstacle_threshold", obstacle_threshold_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "obstacle_threshold");
        
        declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_avoidance_range", rclcpp::ParameterValue(4));
        if (false == node->get_parameter(plugin_name_ + ".obstacle_avoidance_range", obstacle_avoidance_range))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "obstacle_avoidance_range");
        
        declare_parameter_if_not_declared(node, plugin_name_ + ".virtual_laser_range", rclcpp::ParameterValue(60.0));
        if (false == node->get_parameter(plugin_name_ + ".virtual_laser_range", virtual_laser_range))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "virtual_laser_range");

        declare_parameter_if_not_declared(node, plugin_name_ + ".virtual_laser_interval", rclcpp::ParameterValue(20.0));
        if (false == node->get_parameter(plugin_name_ + ".virtual_laser_interval", virtual_laser_interval))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "virtual_laser_interval");

        declare_parameter_if_not_declared(node, plugin_name_ + ".w_min", rclcpp::ParameterValue(-0.6));
        if (false == node->get_parameter(plugin_name_ + ".w_min", w_min_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "w_min");

        declare_parameter_if_not_declared(node, plugin_name_ + ".w_max", rclcpp::ParameterValue(0.6));
        if (false == node->get_parameter(plugin_name_ + ".w_max", w_max_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "w_max");

        declare_parameter_if_not_declared(node, plugin_name_ + ".acc_lin_max", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".acc_lin_max", acc_lin_max_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "acc_lin_max");

        declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_radius", rclcpp::ParameterValue(0.33));
        if (false == node->get_parameter(plugin_name_ + ".wheel_radius", wheel_radius_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "wheel_radius");

        declare_parameter_if_not_declared(node, plugin_name_ + ".track_width", rclcpp::ParameterValue(0.16));
        if (false == node->get_parameter(plugin_name_ + ".track_width", track_width_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "track_width");

        declare_parameter_if_not_declared(node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.1));
        if (false == node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "goal_tolerance");

        declare_parameter_if_not_declared(node, plugin_name_ + ".kp_rot", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".kp_rot", kp_rot_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "kp_rot");

        declare_parameter_if_not_declared(node, plugin_name_ + ".sl_p", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".sl_p", sl_p))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "sl_p");

        declare_parameter_if_not_declared(node, plugin_name_ + ".sl_a", rclcpp::ParameterValue(1.0));
        if (false == node->get_parameter(plugin_name_ + ".sl_a", sl_a))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "sl_a");

        declare_parameter_if_not_declared(node, plugin_name_ + ".l_slack", rclcpp::ParameterValue(0.05));
        if (false == node->get_parameter(plugin_name_ + ".l_slack", l_slack))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "l_slack");
        
        declare_parameter_if_not_declared(node, plugin_name_ + ".acc_slack", rclcpp::ParameterValue(2.0));
        if (false == node->get_parameter(plugin_name_ + ".acc_slack", acc_slack))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "acc_slack");

        declare_parameter_if_not_declared(node, plugin_name_ + ".if_do_goalctrl", rclcpp::ParameterValue(true));
        if (false == node->get_parameter(plugin_name_ + ".if_do_goalctrl", if_do_goalctrl_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "if_do_goalctrl");
        
        declare_parameter_if_not_declared(node, plugin_name_ + ".if_do_failcheck", rclcpp::ParameterValue(true));
        if (false == node->get_parameter(plugin_name_ + ".if_do_failcheck", if_do_failcheck_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "if_do_failcheck");

        // Check parameter consistency
        if (std::fmod(fblin_frequency_, MPC_frequency_)!=0.0) {
            RCLCPP_ERROR(logger_, "MPC_frequency must be a multiple of controller_frequency");
        }

        // Create publishers
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
        int n_side = static_cast<int>(virtual_laser_range / virtual_laser_interval); 
        int n_scan = n_side * 2 + 1; 

        MPCcontroller->set_ErrorMsgCallback(handleErrorMessages);
        MPCcontroller->set_DebugMsgCallback(handleDebugMessages);
        MPCcontroller->set_InfoMsgCallback(handleInfoMessages);

        MPCcontroller->set_MPCparams(1.0/MPC_frequency_, prediction_horizon_, q_, r_, sl_p, sl_a, l_slack, acc_slack, n_scan);
        MPCcontroller->set_FBLINparams(1.0/fblin_frequency_, p_dist_);
        MPCcontroller->set_robotParams(w_max_, w_min_, wheel_radius_, track_width_, acc_lin_max_);

        if (!MPCcontroller->initialize()) {
            RCLCPP_ERROR(logger_, "Unable to initialize MPC controller");
        }

        path_duration_ = path_length_ = 0.0;
        path_time_ = 0;
    }

    void DiffMPC::cleanup() {
        RCLCPP_INFO(logger_, "Cleaning up controller: %s of type tong_controller::DiffMPC", plugin_name_.c_str());

        // Reset publishers
        reference_path_pub_.reset();

        // Delete MPC controller object
        if (MPCcontroller) {
            delete MPCcontroller;
        }
    }

    void DiffMPC::activate() {
        RCLCPP_INFO(logger_, "Activating controller: %s of type tong_controller::DiffMPC", plugin_name_.c_str());

        // Activate publishers
        reference_path_pub_->on_activate();

        // Add callback for dynamic parameters
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(
                std::bind(&DiffMPC::dynamicParametersCallback, this, std::placeholders::_1));
    }

    void DiffMPC::deactivate() {
        RCLCPP_INFO(logger_, "Deactivating controller: %s of type tong_controller::DiffMPC", plugin_name_.c_str());

        // Deactivate publishers
        reference_path_pub_->on_deactivate();

        // Reset dynamic parameter handler
        dyn_params_handler_.reset();
    }

    const std::vector<std::pair<int, int>> neighborSets = {
            {-1, -1},
            { 0, -1},
            { 1, -1},
            { 1,  0},
            { 1,  1},
            { 0,  1},
            {-1,  1},
            {-1,  0}
        };
    geometry_msgs::msg::TwistStamped DiffMPC::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist& /*speed*/, nav2_core::GoalChecker* /*goal_checker*/) {

        std::lock_guard<std::mutex> lock_reinit(mutex_);

        nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;

        // control orientation at goal
        if (if_do_goalctrl_) {
            double goal_dist = nav2_util::geometry_utils::euclidean_distance(pose.pose.position, goal_pose.pose.position);
            if (goal_dist < goal_tolerance_) {
                cmd_vel.twist.linear.x = 0.0;
                double d_theta = tf2::getYaw(goal_pose.pose.orientation) - tf2::getYaw(pose.pose.orientation);
                while (d_theta > M_PI) d_theta -= 2 * M_PI; 
                while (d_theta < -M_PI) d_theta += 2 * M_PI; 
                double angular_vel = kp_rot_ * d_theta;
                double max_vel_angular_ = (w_max_-w_min_)/track_width_*wheel_radius_;
                cmd_vel.twist.angular.z = std::max(-max_vel_angular_, std::min(angular_vel, max_vel_angular_));
                return cmd_vel;
            }
        }        

        // Update controller state
        MPCcontroller->set_actualRobotState(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
        Eigen::VectorXd referenceRobotState;

        // Execute MPC controller
        // TBD: Add replanning in case of too much MPC failures
        if (MPC_execution_counter_ == (int)fblin_frequency_/MPC_frequency_) {
            // Update MPC reference
            // Check that a plan has been received
            if ((path_duration_==0.0) || (path_length_==0.0)) {
                RCLCPP_ERROR(logger_, "No plan available, cannot start the controller");

                // Set the reference equal to the actual pose
                MPCcontroller->set_referenceRobotState(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
            } else {
                // Compute the reference robot state along the prediction horizon
                referenceRobotState.resize(3 * (prediction_horizon_ + 1), 1);
                for (int k = 0; k < prediction_horizon_ + 1; k++) {
                    double x, y, yaw;
                    interpolateTrajectory((path_time_+k)/MPC_frequency_, x, y, yaw);

                    referenceRobotState(3 * k) = x;
                    referenceRobotState(3 * k + 1) = y;
                    referenceRobotState(3 * k + 2) = yaw;
                }

                // Compute obstacle avoidcance constraint
                // obstacle matrix
                unsigned int center_x, center_y;
                double P_x, P_y; 
                P_x = pose.pose.position.x + p_dist_ * cos(tf2::getYaw(pose.pose.orientation));
                P_y = pose.pose.position.y + p_dist_ * sin(tf2::getYaw(pose.pose.orientation));
                costmap->worldToMap(P_x, P_y, center_x, center_y);
                int map_range = 2 * obstacle_avoidance_range + 1;
                int map_center = obstacle_avoidance_range;

                Eigen::MatrixXd obstacle_map = Eigen::MatrixXd::Zero(map_range, map_range);
                for (int ii = 0; ii < map_range; ii++) {
                    for (int jj = 0; jj < map_range; jj++) {
                        unsigned char cost = costmap->getCost(center_x + (ii - map_center), center_y + (jj - map_center));
                        if (static_cast<int>(cost) >= obstacle_threshold_) {
                            obstacle_map(ii, jj) = 1.0;
                        }
                    }
                }

// RCLCPP_INFO(logger_, "DEBUGMARK_LOOP_START");  // for debug only
                // virtual laser search
                int n_side = static_cast<int>(virtual_laser_range / virtual_laser_interval); 
                int n_scan = n_side * 2 + 1; 
                std::vector<double> virtual_laser(n_scan); 
                virtual_laser.at(n_side) = 0.0; 
                for (int ii = 1; ii < n_side + 1; ii++) {
                    virtual_laser.at(n_side - ii) = static_cast<double>(virtual_laser_interval * ii) * M_PI/180;
                    virtual_laser.at(n_side + ii) = -static_cast<double>(virtual_laser_interval * ii) * M_PI/180;
                }
// RCLCPP_INFO(logger_, "DEBUGMARK_LOOP_END");  // for debug only

                std::vector<std::pair<int, int>> scan_result(0);
                for (int ii = 0; ii < n_scan; ii++) {
                    double scan_direction = tf2::getYaw(pose.pose.orientation) + virtual_laser.at(ii);
                    
                    int current_x = map_center; 
                    int current_y = map_center; 
                    int current_length = 1;
                    do {
                        current_x = current_x + static_cast<int>(cos(scan_direction) * current_length);
                        current_y = current_y + static_cast<int>(sin(scan_direction) * current_length);
                        current_length++;

                        if (obstacle_map(current_x, current_y) == 1.0 
                        && current_x >= 0 && current_x < map_range && current_y >= 0 && current_y < map_range) {
                            scan_result.push_back({current_x, current_y});
                            break;
                        }
                    } while (current_x >= 0 && current_x < map_range && current_y >= 0 && current_y < map_range);
                }

                std::vector<std::vector<double>> obstacle_info(0);
                for (auto k = 0; k < scan_result.size(); k++) {
                    int imx = static_cast<int>(center_x) + (scan_result.at(k).first - map_center);
                    int imy = static_cast<int>(center_y) + (scan_result.at(k).second - map_center);
                    std::vector<unsigned int> neighborSetsCost(8);
                    for (int jj = 0; jj < 8; jj++) {
                        int nx1 = imx + neighborSets[jj].first;
                        int ny1 = imy + neighborSets[jj].second;
                        unsigned char cost1 = costmap->getCost(nx1, ny1);
                        neighborSetsCost[jj] = static_cast<int>(cost1);
                    }
                    int count = 0;
                    std::vector<int> neighborSetsIndex(2, 0);
                    for (int jj = 0; jj < 8; jj++) {
                        if (neighborSetsCost[jj] >= obstacle_threshold_) {
                            int jj1 = (jj + 7) % 8;
                            int jj2 = (jj + 1) % 8;
                            if (neighborSetsCost[jj1] < obstacle_threshold_ || neighborSetsCost[jj2] < obstacle_threshold_) {
                                neighborSetsIndex[count] = jj;
                                count++;
                            }
                        }
                    }
                    double Ax = 0.0, Ay = 0.0, B = 0.0;
                    double obstacle_x, obstacle_y;
                    costmap->mapToWorld(imx, imy, obstacle_x, obstacle_y);
                    if (count == 2) {
                        Ax = neighborSets[neighborSetsIndex[0]].second - neighborSets[neighborSetsIndex[1]].second;
                        Ay = neighborSets[neighborSetsIndex[1]].first - neighborSets[neighborSetsIndex[0]].first;
                        B = Ax * obstacle_x + Ay * obstacle_y;

                        double slope_angle = atan2(neighborSets[neighborSetsIndex[0]].second - neighborSets[neighborSetsIndex[1]].second, 
                                            neighborSets[neighborSetsIndex[0]].first - neighborSets[neighborSetsIndex[1]].first) + M_PI/2;
                        int mark_xm = imx + static_cast<int>(1.5 * cos(slope_angle));
                        int mark_ym = imy + static_cast<int>(1.5 * sin(slope_angle));
                        double mark_xw, mark_yw;
                        costmap->mapToWorld(mark_xm, mark_ym, mark_xw, mark_yw);
                        int mark_cost = static_cast<int>(costmap->getCost(mark_xm, mark_ym));
                        if ((Ax * mark_xw + Ay * mark_yw > B && mark_cost < obstacle_threshold_) || 
                                (Ax * mark_xw + Ay * mark_yw <= B && mark_cost >= obstacle_threshold_)) {
                            Ax = -Ax;
                            Ay = -Ay;
                            B = -B;
                        } 

                        obstacle_info.push_back({Ax, Ay, B});
                    }           
                }

                MPCcontroller->set_obstacleConstraint(obstacle_info);

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

            // Execute MPC controller
            if (!MPCcontroller->executeMPCcontroller()) {
                RCLCPP_ERROR(logger_, "No optimal solution found");
                count_infeasible_sol_++;
            } else {
                count_infeasible_sol_ = 0;
            }

            if (if_do_failcheck_) {
                double ref_delay = std::pow(std::pow(pose.pose.position.x - referenceRobotState(0), 2) + std::pow(pose.pose.position.y - referenceRobotState(1), 2), 0.5);
                if (count_infeasible_sol_ >= max_infeasible_sol_ || ref_delay >= max_ref_delay_) {
                    cmd_vel.twist.linear.x = 0.0;
                    cmd_vel.twist.angular.z = 0.0;
                    RCLCPP_ERROR(logger_, "It seems that the trajectory tracking has failed, please try resetting the goal!");
                    return cmd_vel;
                }
            }

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
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    void DiffMPC::setPlan(const nav_msgs::msg::Path &path) {
        RCLCPP_INFO(logger_, "Got new plan");
        goal_pose = path.poses[path.poses.size() - 1];

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

        MPCcontroller->reset_pre_vP();
        count_infeasible_sol_ = 0;
    }

    void DiffMPC::setSpeedLimit(const double &speed_limit, const bool &percentage) {
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

    rcl_interfaces::msg::SetParametersResult DiffMPC::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
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
                } else if (name == plugin_name_ + ".virtual_laser_range") {
                    virtual_laser_range = parameter.as_double();
                } else if (name == plugin_name_ + ".virtual_laser_interval") {
                    virtual_laser_interval = parameter.as_double();
                } else if (name == plugin_name_ + ".goal_tolerance") {
                    goal_tolerance_ = parameter.as_double();
                } else if (name == plugin_name_ + ".kp_rot") {
                    kp_rot_ = parameter.as_double();
                } else if (name == plugin_name_ + ".sl_p") {
                    sl_p = parameter.as_double();
                } else if (name == plugin_name_ + ".sl_a") {
                    sl_a = parameter.as_double();
                } else if (name == plugin_name_ + ".l_slack") {
                    l_slack = parameter.as_double();
                } else if (name == plugin_name_ + ".acc_slack") {
                    acc_slack = parameter.as_double();
                } else if (name == plugin_name_ + ".max_ref_delay") {
                    max_ref_delay_ = parameter.as_double();
                }

            } else if (type == ParameterType::PARAMETER_INTEGER) {
                if (name == plugin_name_ + ".prediction_horizon") {
                    prediction_horizon_ = parameter.as_int();
                } else if (name == plugin_name_ + ".max_infeasible_sol") {
                    max_infeasible_sol_ = parameter.as_int();
                } else if (name == plugin_name_ + ".obstacle_threshold") {
                    obstacle_threshold_ = parameter.as_int();
                } else if (name == plugin_name_ + ".obstacle_avoidance_range") {
                    obstacle_avoidance_range = parameter.as_int();
                }
                
            }
        }

        result.successful = true;
        return result;
    }

    void DiffMPC::interpolateTrajectory(double t, double& x, double& y, double& yaw) {
        double s = path_length_*(3.0*std::pow(std::min(t/path_duration_, 1.0), 2.0)-
                2.0*std::pow(std::min(t/path_duration_, 1.0), 3.0));

        x = gsl_spline_eval(cspline_x, s, acc_x);
        y = gsl_spline_eval(cspline_y, s, acc_y);
        yaw = gsl_spline_eval(cspline_yaw, s, acc_yaw);

    }

}  // namespace tong_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(tong_controller::DiffMPC, nav2_core::Controller)
