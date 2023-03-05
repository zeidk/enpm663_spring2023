cmake_minimum_required(VERSION 3.8)
project(lecture6)

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

include_directories(include/enpm663_ref_frames)

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

add_executable(sensor_camera src/sensor_camera.cpp src/main.cpp)
ament_target_dependencies(sensor_camera ${dependencies})

install(TARGETS
 sensor_camera
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch include config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
