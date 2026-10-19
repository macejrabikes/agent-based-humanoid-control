cmake_minimum_required(VERSION 3.8)
project(humanoid_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pose_skills src/pose_skills.cpp)
ament_target_dependencies(pose_skills rclcpp std_msgs trajectory_msgs ament_index_cpp)
target_link_libraries(pose_skills yaml-cpp::yaml-cpp)

add_executable(motion_skills src/motion_skills.cpp)
target_include_directories(motion_skills PRIVATE include)
ament_target_dependencies(motion_skills rclcpp rclcpp_action std_msgs geometry_msgs
  sensor_msgs nav_msgs humanoid_interfaces)

add_executable(camera_snapshot src/camera_snapshot.cpp)
target_include_directories(camera_snapshot PRIVATE include)
ament_target_dependencies(camera_snapshot rclcpp sensor_msgs std_srvs)

install(TARGETS pose_skills motion_skills camera_snapshot DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
