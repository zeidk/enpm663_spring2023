cmake_minimum_required(VERSION 3.8)
project(first_package_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)


include_directories(include/first_package_cpp)

add_executable(first_node_cpp src/first_node_cpp.cpp)
ament_target_dependencies(first_node_cpp rclcpp)


add_executable(publisher_cpp src/publisher_cpp.cpp)
ament_target_dependencies(publisher_cpp rclcpp example_interfaces)

add_executable(subscriber_cpp src/subscriber_cpp.cpp)
ament_target_dependencies(subscriber_cpp rclcpp example_interfaces)

install(TARGETS
first_node_cpp 
publisher_cpp 
subscriber_cpp
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION share/${PROJECT_NAME})

ament_package()
