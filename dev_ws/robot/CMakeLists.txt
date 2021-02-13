cmake_minimum_required(VERSION 3.5)
project(robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/Robot.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_ros_planning_interface
  robot_msgs
)

add_executable(node src/node.cpp)
ament_target_dependencies(node rclcpp robot_msgs)
target_link_libraries(node ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(robot_test src/test_Robot.cpp)
  ament_target_dependencies(robot_test rclcpp)
  target_link_libraries(robot_test ${PROJECT_NAME})
endif()

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
