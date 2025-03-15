# Appendix - User Manual  
Polimi - M.Sc. Thesis: Developing a MPC controller for autonomous navigation of a differential-drive robot  
Developed by Liu, Tong  
Supervised by Prof. Bascetta, Luca  
## 1. Introduction  
As a result of the thesis work, two ROS2 packages were developed. The link to the GitHub repositories are as follows:  
```
https://github.com/Tong-Liu-6/tong_controller.git
https://github.com/Tong-Liu-6/tong_thesis_1.git
```
The first package is a Nav2 controller plugin with MPC developed in this thesis work. The second package includes files of simulation testings for the developed controller.  
## 2. Setup  
The developed packages must run under the ROS2 framework and Nav2 architecture. Some other library and package dependencies are required, such as Gazebo, Cartographer, TurtleBot3, GSL, and Gurobi.  
Please follow the steps to install the packages:  
1. Download and extract the two package folders to “{your workspace folder}/src/”.  
2. In “{your workspace folder}/src/tong_controller/FindGUROBI.cmake (line 7)”, correctly set the Gurobi version.  
3. In “{your workspace folder}/src/tong_controller/include/MPC_diffDrive_fblin.h (line 10-11)”, correctly set the Gurobi license.   
4. In the terminal open “{your workspace folder}”, and run the following command to compile and build the packages.  
```
colcon build
```
## 3. Mapping  
Please follow the steps to do mapping on the simulation testing environment:  
1. Run the following command in terminal to start the simulation environment in Gazebo and the Cartographer node for SLAM.  
```
export TURTLEBOT3_MODEL=burger
ros2 launch tong_thesis_1 testing_mapping.py
```
2. Run the following command in a new terminal to start a node to control the robot by keyboard.  
```
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```
3. Control the robot to move in the simulation environment, traversing every position as much as possible until the mapping result displayed in RViz is sufficient.  
4. Run the following command in a new terminal to save the generated map files.  
```
ros2 run nav2_map_server map_saver_cli -f {where tong_thesis_1 installed}/maps/map01
```
Generally the path of the installed tong_thesis_1 package is “{your workspace folder}/install/tong_thesis_1/share/tong_thesis_1/”. You can also save the map files elsewhere and then copy it there later. 
## 4. Navigation  
Please follow the steps to start the navigation testing in the simulation environment:  
1. Run the following command in terminal to start all the related parts. 
```
export TURTLEBOT3_MODEL=burger
ros2 launch tong_thesis_1 testing_navigation.py
```
2. Use “2D Pose Estimate” in RViz to set the initial pose of the robot.  
3. Use “Nav2 Goal” in RViz to set the goal of the navigation task, and then the navigation will start.  
4. Two additional obstacle models have been added to the simulation environment, and you can move them to the desired position in Gazebo to test the obstacle avoidance function of the controller.  
## 5. Customization and Configuration  
1. The simulation environment can be defined in a Gazebo model file “{where tong_thesis_1 installed}/worlds/testing_world.model”.  
2. The robot model can be defined in two files. One is a Gazebo model file in“{where tong_thesis_1 installed}/worlds/burger.model”, and the other is a urdf file in “{where tong_thesis_1 installed}/urdf/turtlebot3_burger.urdf”.  
3. Cartographer can be configured in the file “{where tong_thesis_1 installed}/cartographer_config/turtlebot3_lds_2d.lua”.  
4. RViz can be configured in the file “{where tong_thesis_1 installed}/rviz/nav2_default_view.rviz”.  
5. The navigation can be configured in the file “{where tong_thesis_1 installed}/params/controller_params.yaml”.  
6. You can replace the above parts with your own files, as long as they are correctly defined in the launch files “{where tong_thesis_1 installed}/launch/testing_mapping.py” and “{where tong_thesis_1 installed}/launch/testing_navigation.py”.  
## 6. Example of Controller Configuration  
An example (also as an explanation) for the configuration of the MPC controller plugin developed in this thesis work is as follows:  
```
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 50.0 # Feedback Linearization loop frequency
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"] 
    controller_plugins: ["FollowPath"]
    
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.05
      movement_time_allowance: 10.0
      
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.2
      
    FollowPath: 
      plugin: "tong_controller/DiffMPC"

      # parameters of MPC
      prediction_horizon: 10 # MPC prediction horizon
      MPC_frequency: 10.0 # MPC loop frequency
      p_dist: 0.10 # position of point P
      q: 1.0 # cost weight of state error
      r: 3.0 # cost weight of input energy
      sl_p: 100000.0 # cost weight of positon slack variables
      sl_a: 100000.0 # cost weight of acceleration slack variables
      l_slack: 0.03 # slack distance for linear obstacle constraints
      acc_slack: 1.0 # slack value for linear acceleration constraints

      # parameters of goal pose control
      if_do_goalctrl: true # if true, use a proportional control on the robot orientation around the goal position
      goal_tolerance: 0.12 # when the distance between robot and goal is lower than this value, start goal orientation control
      kp_rot: 1.0 # proportional gain for goal orientation control

      # parameters of obstacle avoidance
      obstacle_threshold: 251 # the threshold of cell occupied value (0-255) in local costmap to considered as dangerous region
      obstacle_avoidance_range: 4 # obstacle detection distance of virtual laser 
                                  # (reasonable value: maximum movement distance in prediction_horizon devided by local costmap resolution)
      virtual_laser_range: 60.0 # virtual laser angle range: (-virtual_laser_range) to (+virtual_laser_range)
      virtual_laser_interval: 20.0 # virtual laser interval angle

      # parameters of fail check for trajectory tracking
      if_do_failcheck_: true # if true, enable the fail check process for trajectory tracking in the control loop
      max_infeasible_sol_: 5 # maximum allowable infeasible mpc loop before return fail
      max_ref_delay: 1.0 # maximum allowable distance between robot current position and reference position before return fail
      
      # parameters of robot
      w_min: -6.2 # min allowable wheel actuator rotational speed
      w_max: 6.2 # max allowable wheel actuator rotational speed
      acc_lin_max: 1.0 # max allowable robot linear acceleration
      wheel_radius: 0.033 # robot wheel radius
      track_width: 0.17 # length between left and right wheels
```