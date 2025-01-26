cmake_minimum_required(VERSION 3.5)
project(tong_controller_plugin)

#
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GSL REQUIRED)
#
find_package(GUROBI REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(ALGLIB REQUIRED)

#
# nav2_package()
# set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
  ${GUROBI_INCLUDE_DIRS}
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  tf2_geometry_msgs
  GSL
)

add_library(${library_name} SHARED
  src/trackedmpc.cpp
  src/followtrajectory.cpp
  src/fblin_unicycle.cpp
  src/GUROBIsolver.cpp 
  src/MPC_diffDrive_fblin.cpp
)

ament_target_dependencies(${library_name}
  # PUBLIC
  ${dependencies}
)

# target_link_libraries(${library_name} PRIVATE ALGLIB)
target_link_libraries(${library_name} GSL::gsl GSL::gslcblas Eigen3::Eigen ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY}) # ALGLIB

pluginlib_export_plugin_description_file(nav2_core tong_controller_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# message("${TARGETS}")
install(DIRECTORY include/
  DESTINATION include/
)

install(FILES tong_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)


  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
