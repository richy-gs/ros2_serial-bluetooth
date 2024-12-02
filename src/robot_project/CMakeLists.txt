cmake_minimum_required(VERSION 3.8)
project(robot_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)


# create a executable
add_executable(robot_mover src/robotMover.cpp)
ament_target_dependencies(robot_mover rclcpp geometry_msgs)

add_executable(cmd_vel_turtle src/cmdVelTurtle.cpp)
ament_target_dependencies(cmd_vel_turtle rclcpp geometry_msgs)

add_executable(robot_controller src/robotController.cpp)
ament_target_dependencies(robot_controller rclcpp geometry_msgs tf2_ros tf2 tf2_geometry_msgs)

# Instala los archivos de lanzamiento
install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# install inside the workspace
install(TARGETS
  robot_mover
  cmd_vel_turtle
  robot_controller
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
