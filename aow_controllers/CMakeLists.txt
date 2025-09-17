cmake_minimum_required(VERSION 3.16)
project(aow_controllers)

# Warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ==============================================================================
# FIND DEPENDENCIES
# ==============================================================================

# LibTorch (optional)
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtorch;${CMAKE_PREFIX_PATH}")
find_package(Torch QUIET)
if(Torch_FOUND)
  message(STATUS "PyTorch found. Building with PyTorch JIT/CUDA support.")
  add_definitions(-DTORCH_AVAILABLE)
else()
  message(WARNING "PyTorch NOT found. Building without Torch/JIT support.")
endif()

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# YAML-CPP (via pkg-config)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

# ==============================================================================
# EXECUTABLES
# ==============================================================================

set(COMMON_DEPENDENCIES
  rclcpp
  rcl_interfaces
  ament_index_cpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  ros_gz_interfaces
)

add_executable(aow_controller_executor
  src/controller_executor.cpp
  src/example_controllers/end2end/end2end_controller_class.cpp
  src/example_controllers/pure_pursuit/pure_pursuit_controller_class.cpp
  src/example_controllers/custom/custom_controller_class.cpp
  src/utils/trajectory_utils.cpp
)

target_include_directories(aow_controller_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(aow_controller_executor PUBLIC cxx_std_17)

# Link ROS deps (this uses the PLAIN signature internally)
ament_target_dependencies(aow_controller_executor ${COMMON_DEPENDENCIES})

# Link YAML-CPP (PLAIN signature)
target_link_libraries(aow_controller_executor ${YAML_CPP_LIBRARIES})
target_include_directories(aow_controller_executor PRIVATE ${YAML_CPP_INCLUDE_DIRS})

# Link LibTorch if found (PLAIN signature)
if(Torch_FOUND)
  target_link_libraries(aow_controller_executor ${TORCH_LIBRARIES})
  # Keep these—unrelated to the signature issue
  target_compile_definitions(aow_controller_executor PRIVATE _GLIBCXX_USE_CXX11_ABI=1)
  target_link_options(aow_controller_executor PRIVATE "-Wl,--no-as-needed")
endif()

# ==============================================================================
# INSTALL
# ==============================================================================
install(TARGETS aow_controller_executor DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz/)

install(PROGRAMS
  src/utils/ps5_twist_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/utils/trajectory_generation/example_trajectory_csv/
  DESTINATION share/${PROJECT_NAME}/utils/trajectory_generation/example_trajectory_csv/
  FILES_MATCHING PATTERN "*.csv"
)

install(DIRECTORY src/example_controllers/end2end/end2end_policy/
  DESTINATION share/${PROJECT_NAME}/example_controllers/end2end/end2end_policy/
  FILES_MATCHING PATTERN "*.pt" PATTERN "*.jit"
)

install(DIRECTORY src/example_controllers/pure_pursuit/locomotion_policy/
  DESTINATION share/${PROJECT_NAME}/example_controllers/pure_pursuit/locomotion_policy/
  OPTIONAL
  FILES_MATCHING PATTERN "*.pt" PATTERN "*.jit"
)

# ==============================================================================
# TESTING
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
