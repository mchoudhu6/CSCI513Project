cmake_minimum_required(VERSION 3.5)
project(act)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install python file
install(
  PROGRAMS
    path_finding_algo.py 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Gazebo world and models
install(DIRECTORY worlds/ models/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
