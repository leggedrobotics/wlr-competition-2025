cmake_minimum_required(VERSION 3.8)
project(aow_sim_worlds)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY models worlds
  DESTINATION share/${PROJECT_NAME}
)

# (Optional) Install environment hooks for GZ_SIM_RESOURCE_PATH
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/env_hooks")
  install(
    DIRECTORY env_hooks
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}.dsv.in")

add_executable(publish_mesh_node src/publish_mesh_node.cpp)
ament_target_dependencies(publish_mesh_node rclcpp visualization_msgs)

install(TARGETS publish_mesh_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
