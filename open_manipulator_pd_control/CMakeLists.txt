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
  test_client.py
  automated_test.py
  plot_pd_results.py
  set_current_mode.py
  helper.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install MATLAB scripts and documentation
install(FILES
  plot_pd_results.m
  README.md
  QUICKSTART.md
  PACKAGE_SUMMARY.md
  STRUCTURE.md
  TROUBLESHOOTING.md
  FINAL_SUMMARY.md
  INDEX.md
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
