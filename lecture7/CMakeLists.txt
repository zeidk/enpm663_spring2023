cmake_minimum_required(VERSION 3.8)
project(lecture7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include/lecture7)

set(dependencies
  rclcpp
  std_srvs
  ariac_msgs
  geometry_msgs
  tf2  
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
)

add_executable(lecture7_exe src/sensor_camera.cpp src/main.cpp)
ament_target_dependencies(lecture7_exe ${dependencies})

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/main.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  lecture7_exe
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch include config rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
