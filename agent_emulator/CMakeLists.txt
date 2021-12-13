cmake_minimum_required(VERSION 3.5)
project(agent_emulator)

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
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Generate interfaces (services)
# find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetPose.srv
  srv/SetVelocity.srv
  srv/SetAcceleration.srv
  action/GoTo.action
  DEPENDENCIES geometry_msgs
)


# Add Agent target library
add_library(agent_node SHARED
  src/agent_emulator/agent.cpp
)
target_include_directories(agent_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(agent_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  rclcpp_action
  rclcpp_components
  rcl_interfaces
)
rosidl_target_interfaces(agent_node ${PROJECT_NAME} rosidl_typesupport_cpp)
rclcpp_components_register_node(agent_node
  PLUGIN "agent_emulator::Agent"
  EXECUTABLE agent
)

# Add AgentViz target library
add_library(agent_viz_node SHARED
  src/agent_emulator/agent_viz.cpp
)
target_include_directories(agent_viz_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(agent_viz_node
  rclcpp
  rclcpp_components
  geometry_msgs
  visualization_msgs
)
rclcpp_components_register_node(agent_viz_node
  PLUGIN "agent_emulator::AgentViz"
  EXECUTABLE agent_viz
)

# Install node libraries
install(TARGETS
  agent_node
  agent_viz_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install node executables
install(TARGETS
  agent
  agent_viz
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install meshes directory
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}/
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
  agent_node
  agent_viz_node
)

ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_action
  rclcpp_components
  rcl_interfaces
  tf2
  tf2_ros
  geometry_msgs
  visualization_msgs
)

ament_package()
