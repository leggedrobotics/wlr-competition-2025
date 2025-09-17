cmake_minimum_required(VERSION 3.8)
project(aow_gazebo_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# Install launch/config/world files
install(DIRECTORY
  launch
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(
  ros_gz_sim
  ros_gz_bridge
)

ament_package()
