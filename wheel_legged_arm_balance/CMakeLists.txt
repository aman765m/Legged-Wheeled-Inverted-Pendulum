cmake_minimum_required(VERSION 3.8)
project(wheel_legged_arm_balance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
set(THIS_PACKAGE_DEPENDS
  rclcpp rclcpp_lifecycle controller_interface hardware_interface pluginlib realtime_tools
  geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs trajectory_msgs visualization_msgs
  tf2_ros)
foreach(dep IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# ---------------- ros2_control controllers (plugin library) ----------------
add_library(wheel_legged_arm_balance_controllers SHARED
  src/lqr_balance_controller.cpp
  src/rac_arm_controller.cpp)
target_include_directories(wheel_legged_arm_balance_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(wheel_legged_arm_balance_controllers ${THIS_PACKAGE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface wheel_legged_arm_balance_controllers.xml)

# ---------------- nodes ----------------
add_executable(arm_fsm_node src/arm_fsm_node.cpp)
target_include_directories(arm_fsm_node PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(arm_fsm_node rclcpp geometry_msgs sensor_msgs std_msgs trajectory_msgs)

add_executable(cmp_estimator_node src/cmp_estimator_node.cpp)
target_include_directories(cmp_estimator_node PRIVATE include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(cmp_estimator_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  std_srvs visualization_msgs tf2_ros)
target_link_libraries(cmp_estimator_node urdfdom::urdfdom_model)

install(TARGETS wheel_legged_arm_balance_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS arm_fsm_node cmp_estimator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/generate_meshes.py
  scripts/generate_worlds.py
  scripts/push_disturbance.py
  scripts/infinity_loop_tracker.py
  scripts/experiment_logger.py
  scripts/analyze_results.py
  scripts/run_push_campaign.py
  scripts/compute_lqr.py
  scripts/sim_starter.py
  scripts/reset_sim.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_models test/test_models.cpp)
  target_include_directories(test_models PRIVATE include ${EIGEN3_INCLUDE_DIR})
endif()

ament_export_include_directories(include)
ament_export_libraries(wheel_legged_arm_balance_controllers)
ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_package()
