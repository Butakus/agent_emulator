cmake_minimum_required(VERSION 3.5)
project(agent_velocity_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add RandomVelocityController target library
add_library(random_velocity_controller_node SHARED
  src/random_velocity_controller.cpp
)
target_include_directories(random_velocity_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(random_velocity_controller_node
  rclcpp
  rclcpp_components
  rcl_interfaces
  geometry_msgs
)
rclcpp_components_register_node(random_velocity_controller_node
  PLUGIN "agent_velocity_controller::RandomVelocityController"
  EXECUTABLE random_velocity_controller
)

# Add RandomAccelerationController target library
add_library(random_acceleration_controller_node SHARED
  src/random_acceleration_controller.cpp
)
target_include_directories(random_acceleration_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(random_acceleration_controller_node
  rclcpp
  rclcpp_components
  rcl_interfaces
  geometry_msgs
)
rclcpp_components_register_node(random_acceleration_controller_node
  PLUGIN "agent_velocity_controller::RandomAccelerationController"
  EXECUTABLE random_acceleration_controller
)

# Add TriangularVelocityController target library
add_library(triangular_velocity_controller_node SHARED
  src/triangular_velocity_controller.cpp
)
target_include_directories(triangular_velocity_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(triangular_velocity_controller_node
  rclcpp
  rclcpp_components
  rcl_interfaces
  geometry_msgs
)
rclcpp_components_register_node(triangular_velocity_controller_node
  PLUGIN "agent_velocity_controller::TriangularVelocityController"
  EXECUTABLE triangular_velocity_controller
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(agent_velocity_controller PRIVATE "AGENT_velocity_CONTROLLER_BUILDING_LIBRARY")

# Install library targets
install(TARGETS
  random_velocity_controller_node
  random_acceleration_controller_node
  triangular_velocity_controller_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  random_velocity_controller
  random_acceleration_controller
  triangular_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  random_velocity_controller_node
  random_acceleration_controller_node
  triangular_velocity_controller_node
)

ament_package()
