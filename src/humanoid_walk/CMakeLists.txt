cmake_minimum_required(VERSION 3.8)
project(humanoid_walk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(walk_policy_node src/walk_policy_node.cpp)
target_include_directories(walk_policy_node PRIVATE src)
ament_target_dependencies(walk_policy_node rclcpp sensor_msgs geometry_msgs std_msgs
  control_msgs trajectory_msgs ament_index_cpp nav_msgs controller_manager_msgs)

# overenie C++ politiky proti referencnym vystupom z PyTorchu (bez ROS)
add_executable(g1_policy_check src/g1_policy_check.cpp)
target_include_directories(g1_policy_check PRIVATE src)

# overenie C++ politiky vstavania proti PyTorchu (bez ROS)
add_executable(getup_policy_check src/getup_policy_check.cpp)
target_include_directories(getup_policy_check PRIVATE src)

install(TARGETS walk_policy_node g1_policy_check getup_policy_check DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
