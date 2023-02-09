cmake_minimum_required(VERSION 3.8)
project(first_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)

###########
# C++
###########
include_directories(include/first_package)

add_executable(publisher_cpp src/publisher_cpp.cpp)
ament_target_dependencies(publisher_cpp rclcpp example_interfaces)

add_executable(subscriber_cpp src/subscriber_cpp.cpp)
ament_target_dependencies(subscriber_cpp rclcpp example_interfaces)

install(TARGETS
publisher_cpp 
subscriber_cpp
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include launch DESTINATION share/${PROJECT_NAME})

###########
# Python
###########
# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  nodes/publisher_py.py
  nodes/subscriber_py.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
