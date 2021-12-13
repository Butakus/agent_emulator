cmake_minimum_required(VERSION 3.5)

project(noisy_localization)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# Add noisy_localization node library
add_library(noisy_localization_node SHARED
  src/noisy_localization.cpp
)
target_include_directories(noisy_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(noisy_localization_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(noisy_localization_node
  PLUGIN "noisy_localization::NoisyLocalization"
  EXECUTABLE noisy_localization
)

# Add noisy_platoon_localization node library
add_library(noisy_platoon_localization_node SHARED
  src/noisy_platoon_localization.cpp
)
target_include_directories(noisy_platoon_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(noisy_platoon_localization_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
)
rclcpp_components_register_node(noisy_platoon_localization_node
  PLUGIN "noisy_localization::NoisyPlatoonLocalization"
  EXECUTABLE noisy_platoon_localization
)

# Add noisy_platoon_odometry node library
add_library(noisy_platoon_odometry_node SHARED
  src/noisy_platoon_odometry.cpp
)
target_include_directories(noisy_platoon_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(noisy_platoon_odometry_node
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2_ros
)
rclcpp_components_register_node(noisy_platoon_odometry_node
  PLUGIN "noisy_localization::NoisyPlatoonOdometry"
  EXECUTABLE noisy_platoon_odometry
)

# Add platoon_noise_variator node library
add_library(platoon_noise_variator_node SHARED
  src/platoon_noise_variator.cpp
)
target_include_directories(platoon_noise_variator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(platoon_noise_variator_node
  rclcpp
  rclcpp_components
  geometry_msgs
)
rclcpp_components_register_node(platoon_noise_variator_node
  PLUGIN "noisy_localization::PlatoonNoiseVariator"
  EXECUTABLE platoon_noise_variator
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# target_compile_definitions(noisy_localization PRIVATE "NOISY_LOCALIZATION_BUILDING_LIBRARY")

# Install node libraries
install(TARGETS
  noisy_localization_node
  noisy_platoon_localization_node
  noisy_platoon_odometry_node
  platoon_noise_variator_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install node executables
install(TARGETS
  noisy_localization
  noisy_platoon_localization
  noisy_platoon_odometry
  platoon_noise_variator
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
  noisy_localization_node
  noisy_platoon_localization_node
  noisy_platoon_odometry_node
  platoon_noise_variator_node
)

ament_package()
