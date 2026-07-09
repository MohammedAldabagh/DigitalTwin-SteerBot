cmake_minimum_required(VERSION 3.8)
project(piper_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(piper_demo_node src/piper_demo_node.cpp)
ament_target_dependencies(piper_demo_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(piper_pick_place_node src/piper_pick_place_node.cpp)
ament_target_dependencies(piper_pick_place_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  control_msgs
  trajectory_msgs
)

add_executable(piper_position_sensor_node src/piper_position_sensor_node.cpp)
ament_target_dependencies(piper_position_sensor_node
  rclcpp
  geometry_msgs
  tf2_ros
)

add_library(piper_grab_rotate_lib
  src/piper_grab_rotate.cpp
)

target_include_directories(piper_grab_rotate_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(piper_grab_rotate_lib
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core          
  tf2
  tf2_ros              
  tf2_geometry_msgs
)


add_executable(piper_grab_rotate_node
  src/piper_grab_rotate_main.cpp
)

target_link_libraries(piper_grab_rotate_node
  piper_grab_rotate_lib
)

ament_target_dependencies(piper_grab_rotate_node
  rclcpp
)

install(TARGETS
  piper_demo_node
  piper_pick_place_node
  piper_position_sensor_node
  piper_grab_rotate_lib
  piper_grab_rotate_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
