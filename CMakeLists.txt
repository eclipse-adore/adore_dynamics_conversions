cmake_minimum_required(VERSION 3.8)
project(adore_dynamics_conversions VERSION 0.0.1 LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenDrive REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_dynamics REQUIRED)
find_package(adore_map_conversions REQUIRED)
find_package(adore_math_conversions REQUIRED)
find_package(adore_ros2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/*.c")

# Define the library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link and export dependencies
ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  osqp
  OsqpEigen
  OpenDrive
  adore_math
  adore_dynamics
  adore_map_conversions
  adore_math_conversions
  adore_ros2_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Export CMake config for downstream find_package()
install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# ament exports
ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3 osqp OsqpEigen OpenDrive 
  adore_math adore_dynamics adore_map_conversions adore_math_conversions
  adore_ros2_msgs rclcpp std_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs
)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
