cmake_minimum_required(VERSION 3.8)
project(bagcreator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(bagcreator src/txt2bag.cpp)
ament_target_dependencies(bagcreator rclcpp sensor_msgs geometry_msgs rosbag2_cpp tf2_geometry_msgs)
install(TARGETS bagcreator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
