cmake_minimum_required(VERSION 3.8)
project(micromouse)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
# find_package(rclcpp_components REQUIRED)

include_directories(
  include
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

set(EXEC_NAME "micromouse")

add_executable(${EXEC_NAME} src/micromouse.cpp src/cmdvel.cpp src/encoder.cpp src/imu.cpp src/range_sensor.cpp
src/solver_utils.cpp src/math_utils.cpp src/pose2.cpp src/direction.cpp src/vector2.cpp src/vector2int.cpp
src/angle_utils.cpp src/maze.cpp src/cell.cpp src/solver.cpp src/maze_utils.cpp
src/bitwise_utils.cpp src/robot.cpp src/dc_motor.cpp)
ament_target_dependencies(${EXEC_NAME} ${dependencies})

install(
  DIRECTORY launch worlds urdf params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
