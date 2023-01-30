cmake_minimum_required(VERSION 3.5)
project(lidar_ros2)

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
find_package(lidar_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


include_directories(
  include
  include/sf45
  include/sf45/linux
)


add_library(lwNx src/sf45/lwNx.cpp)
add_library(platformLinux src/sf45/linux/platformLinux.cpp)
add_library(serialPortLinux src/sf45/linux/lwSerialPortLinux.cpp)


# Build

add_executable(sf45b src/sf45/sf45b.cpp)
ament_target_dependencies(sf45b rclcpp lidar_interface sensor_msgs)

add_executable(lidar_parser src/lidar_parser.cpp)
ament_target_dependencies(lidar_parser sensor_msgs rclcpp)

add_executable(lidar_commander src/lidar_commander.cpp)
ament_target_dependencies(lidar_commander lidar_interface rclcpp)

# target_include_directories(sf40c PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lightwarelidar2>
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/lightwarelidar2/linux>
#   $<INSTALL_INTERFACE:include>)

target_link_libraries(sf45b
  lwNx
  platformLinux
  serialPortLinux
)


# Install

install(TARGETS
  sf45b
  lidar_parser
  lidar_commander
  DESTINATION lib/${PROJECT_NAME})


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

ament_package()
