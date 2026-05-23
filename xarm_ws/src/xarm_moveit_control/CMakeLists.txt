cmake_minimum_required(VERSION 3.8)
project(xarm_moveit_control)

# ===============================
# Compiler settings
# ===============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# Dependencies
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xarm_planner REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ===============================
# Locate xarm_planner shared library manually (as in old workspace)
# ===============================
find_library(XARM_PLANNER_LIB
  NAMES libxarm_planner.so xarm_planner
  PATHS
    ${CMAKE_INSTALL_PREFIX}/lib
    /home/hayashi/tomato2025_ws/xarm_ws/install/xarm_planner/lib
    ${xarm_planner_DIR}/../../../lib
  NO_DEFAULT_PATH
)
message(STATUS "XARM_PLANNER_LIB=${XARM_PLANNER_LIB}")

# ===============================
# Include directories
# ===============================
include_directories(
  include
  /home/hayashi/tomato2025_ws/xarm_ws/src/xarm_ros2/xarm_planner/include
)

# ===============================
# Testing (optional)
# ===============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ===============================
# === Executable: minimal_xarm7_moveit_test ===
# ===============================
add_executable(minimal_xarm7_moveit_test src/minimal_xarm7_moveit_test.cpp)
target_compile_features(minimal_xarm7_moveit_test PUBLIC cxx_std_17)
target_include_directories(minimal_xarm7_moveit_test PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${xarm_planner_INCLUDE_DIRS}
  ${xarm_msgs_INCLUDE_DIRS}
)
ament_target_dependencies(minimal_xarm7_moveit_test
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  xarm_planner
  xarm_msgs
  tf2
  tf2_geometry_msgs
  control_msgs
  trajectory_msgs
)
target_link_libraries(minimal_xarm7_moveit_test
  ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${XARM_PLANNER_LIB}
)

# ===============================
# === Executable: motion_planning_node ===
# ===============================
add_executable(motion_planning_node src/motion_planning_node.cpp)
target_compile_features(motion_planning_node PUBLIC cxx_std_17)
target_include_directories(motion_planning_node PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${xarm_planner_INCLUDE_DIRS}
  ${xarm_msgs_INCLUDE_DIRS}
)
ament_target_dependencies(motion_planning_node
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  xarm_planner
  xarm_msgs
  tf2
  tf2_geometry_msgs
  control_msgs
  trajectory_msgs
)
target_link_libraries(motion_planning_node
  ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${XARM_PLANNER_LIB}
)

# ===============================
# === Executable: test_xarm_sdk_motion ===
# ===============================
add_executable(test_xarm_sdk_motion src/test_xarm_sdk_motion.cpp)
target_compile_features(test_xarm_sdk_motion PUBLIC cxx_std_17)
target_include_directories(test_xarm_sdk_motion PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${xarm_msgs_INCLUDE_DIRS}
)
ament_target_dependencies(test_xarm_sdk_motion
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  xarm_msgs
)
target_link_libraries(test_xarm_sdk_motion
  ${rclcpp_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)


# ===============================
# Install targets
# ===============================
install(TARGETS
  minimal_xarm7_moveit_test
  motion_planning_node
  test_xarm_sdk_motion
  DESTINATION lib/${PROJECT_NAME}
)

# ===============================
# Export info for downstream packages
# ===============================
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(xarm_planner)
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  xarm_msgs
)

# ===============================
# Install Python scripts
# ===============================
install(PROGRAMS
  scripts/set_tcp_offset.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
