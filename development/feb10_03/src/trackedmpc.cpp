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

        declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_threshold", rclcpp::ParameterValue(250));
        if (false == node->get_parameter(plugin_name_ + ".obstacle_threshold", obstacle_threshold_))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "obstacle_threshold");
        
        declare_parameter_if_not_declared(node, plugin_name_ + ".obstacle_avoidance_range", rclcpp::ParameterValue(4));
        if (false == node->get_parameter(plugin_name_ + ".obstacle_avoidance_range", obstacle_avoidance_range))
            RCLCPP_ERROR(logger_, "Node %s: unable to retrieve parameter %s.", node->get_name(), "obstacle_avoidance_range");

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
        reference_path_pub_.reset();

        // Delete MPC controller object
        if (MPCcontroller) {
            delete MPCcontroller;
        }
    }

    void TrackedMPC::activate() {
        RCLCPP_INFO(logger_, "Activating controller: %s of type tong_controller::TrackedMPC", plugin_name_.c_str());

        // Activate publishers
        reference_path_pub_->on_activate();

        // Add callback for dynamic parameters
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(
                std::bind(&TrackedMPC::dynamicParametersCallback, this, std::placeholders::_1));
    }

    void TrackedMPC::deactivate() {
        RCLCPP_INFO(logger_, "Deactivating controller: %s of type tong_controller::TrackedMPC", plugin_name_.c_str());

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
    geometry_msgs::msg::TwistStamped TrackedMPC::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist& /*speed*/, nav2_core::GoalChecker* /*goal_checker*/) {

        std::lock_guard<std::mutex> lock_reinit(mutex_);

        nav2_costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

        // RCLCPP_INFO(logger_, "Costmap name: %s", costmap_ros_->getName().c_str());
        // unsigned int mx, my;
        // double wx = pose.pose.position.x, wy = pose.pose.position.y;  
        // costmap->worldToMap(wx, wy, mx, my);  
        // unsigned char cost = costmap->getCost(mx, my); 
        // RCLCPP_INFO(logger_, "Cost: %u", static_cast<unsigned int>(cost));

        // Update controller state
        MPCcontroller->set_actualRobotState(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));

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
                Eigen::VectorXd referenceRobotState;
                referenceRobotState.resize(3 * (prediction_horizon_ + 1), 1);
                for (int k = 0; k < prediction_horizon_ + 1; k++) {
                    double x, y, yaw;
                    interpolateTrajectory((path_time_+k)/MPC_frequency_, x, y, yaw);

                    referenceRobotState(3 * k) = x;
                    referenceRobotState(3 * k + 1) = y;
                    referenceRobotState(3 * k + 2) = yaw;
                }



                // Compute Obstacle Avoidcance Constraint
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

                Eigen::MatrixXd visited_map = Eigen::MatrixXd::Constant(map_range, map_range, -1.0);
                std::vector<int> record_dist(0);
                std::vector<std::pair<int, int>> record_position(0);
                int current_regionID = 0;
                for (int ii = 0; ii < map_range; ii++) {
                    for (int jj = 0; jj < map_range; jj++) {
                        if (visited_map(ii, jj) < 0 && obstacle_map(ii, jj) == 1.0) {
                            record_dist.push_back(map_range * map_range);
                            record_position.push_back({-1, -1}); 
                            searchMap(ii, jj, current_regionID, obstacle_map, visited_map, record_dist, record_position);
                            current_regionID++;
                        }
                    }
                }

                int n_obstacle = record_position.size();
                Eigen::MatrixXd obstacle_info;
                obstacle_info.resize(n_obstacle, 3);
                for (int ii = 0; ii < n_obstacle; ii++) {
                    int imx = static_cast<int>(center_x) + (record_position.at(ii).first - map_center);
                    int imy = static_cast<int>(center_y) + (record_position.at(ii).second - map_center);
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
                    } else if (count == 1) {
                        Ax = neighborSets[neighborSetsIndex[0]].second - 0;
                        Ay = 0 - neighborSets[neighborSetsIndex[0]].first;
                        B = Ax * obstacle_x + Ay * obstacle_y;
                    } else {
                        // Ax = imx - static_cast<int>(center_x); 
                        // Ay = imy - static_cast<int>(center_y); 
                        // B = Ax * obstacle_x + Ay * obstacle_y;
                        Ax = 0.0;
                        Ay = 0.0;
                        B = +INFINITY;
                    }
                    if (Ax * pose.pose.position.x + Ay * pose.pose.position.y > B) {
                        Ax = -Ax;
                        Ay = -Ay;
                        B = -B;
                    }
                    obstacle_info(ii, 0) = Ax;
                    obstacle_info(ii, 1) = Ay;
                    obstacle_info(ii, 2) = B;
                }

                // for (int i = 0; i < obstacle_info.rows(); i++) {
                //     RCLCPP_ERROR(logger_, "OBSTACLE INFO: %d, %f, %f, %f, %d, %d", static_cast<int>(obstacle_info.rows()), obstacle_info(i, 0), obstacle_info(i, 1), obstacle_info(i, 2), record_position[i].first, record_position[i].second);
                // }
                

                MPCcontroller->set_obstacleConstraint(obstacle_info);

                // int obstacle_flag = 0, obstacle_index = -1;
                // double obstacle_x, obstacle_y;
                // double Ax = 0.0, Ay = 0.0, B = 0.0;
                // unsigned int mx, my;
                // for (int k = 0; k < prediction_horizon_ + 1; k++) {
                //     double wx = previous_prediction(2*k); 
                //     double wy = previous_prediction(2*k+1);
                //     costmap->worldToMap(wx, wy, mx, my);  
                //     unsigned char cost = costmap->getCost(mx, my);
                //     if (static_cast<unsigned int>(cost) >= obstacle_threshold_) {
                //         obstacle_flag = 1;
                //         obstacle_index = k;
                //         obstacle_x = wx; 
                //         obstacle_y = wy;
                //         break;
                //     }
                // }
                // if (obstacle_flag == 0) {
                //     MPCcontroller->set_obstacleConstraint(obstacle_flag, Ax, Ay, B);
                // } else {
                //     int imx = static_cast<int>(mx);
                //     int imy = static_cast<int>(my);
                //     std::vector<std::pair<int, int>> neighborSets = {
                //         {-1, -1},
                //         { 0, -1},
                //         { 1, -1},
                //         { 1,  0},
                //         { 1,  1},
                //         { 0,  1},
                //         {-1,  1},
                //         {-1,  0}
                //     };
                //     std::vector<unsigned int> neighborSetsCost(8);
                //     for (int ii = 0; ii < 8; ii++) {
                //         int nx1 = imx + neighborSets[ii].first;
                //         int ny1 = imy + neighborSets[ii].second;
                //         unsigned char cost1 = costmap->getCost(nx1, ny1);
                //         neighborSetsCost[ii] = static_cast<unsigned int>(cost1);
                //     }
                //     int count = 0;
                //     std::vector<int> neighborSetsIndex(2, 0);
                //     for (int ii = 0; ii < 8; ii++) {
                //         if (neighborSetsCost[ii] >= obstacle_threshold_) {
                //             int jj1 = (ii + 7) % 8;
                //             int jj2 = (ii + 1) % 8;
                //             if (neighborSetsCost[jj1] < obstacle_threshold_ || neighborSetsCost[jj2] < obstacle_threshold_) {
                //                 neighborSetsIndex[count] = ii;
                //                 count++;
                //             }
                //         }
                //     }
                //     Ax = neighborSets[neighborSetsIndex[0]].second - neighborSets[neighborSetsIndex[1]].second;
                //     Ay = neighborSets[neighborSetsIndex[1]].first - neighborSets[neighborSetsIndex[0]].first;
                //     B = Ax * obstacle_x + Ay * obstacle_y;
                //     if (Ax * pose.pose.position.x + Ay * pose.pose.position.y > B) obstacle_flag = -1;

                //     // keep previous constraints if count != 2
                //     if (count == 2) MPCcontroller->set_obstacleConstraint(obstacle_flag, Ax, Ay, B);
                    
                // }






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
        RCLCPP_INFO(logger_, "Got new plan");

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
                }
            } else if (type == ParameterType::PARAMETER_INTEGER) {
                if (name == plugin_name_ + ".prediction_horizon") {
                    prediction_horizon_ = parameter.as_int();
                } else if (name == plugin_name_ + ".max_infeasible_sol") {
                    max_infeasible_sol_ = parameter.as_int();
                } else if (name == plugin_name_ + ".obstacle_threshold") {
                    obstacle_threshold_ = parameter.as_int();
                } else if (name == plugin_name_ + ".obstacle_avoidance_range") {
                    obstacle_threshold_ = parameter.as_int();
                }
            }
        }

        result.successful = true;
        return result;
    }

    void TrackedMPC::interpolateTrajectory(double t, double& x, double& y, double& yaw) {
        // double s = path_length_*(3.0*std::pow(std::min(t/path_duration_, path_duration_), 2.0)-
        //         2.0*std::pow(std::min(t/path_duration_, path_duration_), 3.0));
        double s = path_length_*(3.0*std::pow(std::min(t/path_duration_, 1.0), 2.0)-
                2.0*std::pow(std::min(t/path_duration_, 1.0), 3.0));

        // x = spline1dcalc(path_sp_x_, s);
        // y = spline1dcalc(path_sp_y_, s);
        // yaw = spline1dcalc(path_sp_yaw_, s);

        x = gsl_spline_eval(cspline_x, s, acc_x);
        y = gsl_spline_eval(cspline_y, s, acc_y);
        yaw = gsl_spline_eval(cspline_yaw, s, acc_yaw);

    }

    const int dx[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dy[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    void TrackedMPC::searchMap(int i, int j, int regionID, const Eigen::MatrixXd &obstacle_map, Eigen::MatrixXd &visited_map, 
                            std::vector<int> &record_dist, std::vector<std::pair<int, int>> &record_position) {
        int rows = obstacle_map.rows();
        int cols = obstacle_map.cols();
        if (i < 0 || i >= rows || j < 0 || j >= cols) return;
        if (visited_map(i, j) >= 0 || obstacle_map(i, j) == 0) return;

        int dist_center = (i - obstacle_avoidance_range) * (i - obstacle_avoidance_range) + (j - obstacle_avoidance_range) * (j - obstacle_avoidance_range);
        visited_map(i, j) = dist_center;
        if (dist_center < record_dist.at(regionID)) {
            record_dist.at(regionID) = dist_center;
            record_position.at(regionID).first = i;
            record_position.at(regionID).second = j;
        }

        for (int k = 0; k < 8; k++) {
            int new_i = i + dx[k];
            int new_j = j + dy[k];
            searchMap(new_i, new_j, regionID, obstacle_map, visited_map, record_dist, record_position);
        }
    }

     

}  // namespace tong_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(tong_controller::TrackedMPC, nav2_core::Controller)
