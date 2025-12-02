cmake_minimum_required(VERSION 3.8)
project(open_manipulator_pd_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetJointPosition.srv"
  DEPENDENCIES std_msgs
)

# Install Python scripts
install(PROGRAMS
  pd_controller_node.py
  pd_controller_direct.py
  test_client.py
  set_current_mode.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install MATLAB script
install(FILES
  plot_pd_results.m
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
