cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${Autonomous Docking Using ArUco Markers in ROS2}
)

# Install URDF files
install(DIRECTORY urdf
  DESTINATION share/${Autonomous Docking Using ArUco Markers in ROS2}
)

# Install RViz configs
install(DIRECTORY rviz
  DESTINATION share/${Autonomous Docking Using ArUco Markers in ROS2}
)

ament_package()
