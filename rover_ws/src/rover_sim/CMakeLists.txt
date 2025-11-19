cmake_minimum_required(VERSION 3.8)
project(rover_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF, meshes, worlds
install(DIRECTORY urdf meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/random_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/random_goal_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/astar_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/random_goal_publisher.py
  scripts/astar_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/potential_field_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
