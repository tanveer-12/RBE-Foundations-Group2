cmake_minimum_required(VERSION 3.8)
project(openmx_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InverseKinematics.srv"
  "srv/EE2JointVel.srv"
  "srv/JointVel2EE.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()