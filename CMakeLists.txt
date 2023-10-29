cmake_minimum_required(VERSION 3.8)
project(line_detection)


#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/line_detection)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY include/line_detection
  DESTINATION include
)


add_library(line_functions src/line_functions.cpp include/line_detection/line_functions.h)
ament_export_targets(line_functions HAS_LIBRARY_TARGET)


install(
  TARGETS line_functions 
  EXPORT line_functions
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/line_detection
)

add_library(Lines src/Lines.cpp include/line_detection/Lines.h)
ament_export_targets(Lines HAS_LIBRARY_TARGET)


install(
  TARGETS Lines 
  EXPORT Lines
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/line_detection
)


ament_export_include_directories(
  include
)

add_executable(simple_lidar_reader src/simple_lidar_reader.cpp)
ament_target_dependencies(simple_lidar_reader rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(save_single_measurement src/save_single_measurement.cpp)
ament_target_dependencies(save_single_measurement rclcpp std_msgs sensor_msgs)

add_executable(publish_lines src/publish_lines.cpp)
ament_target_dependencies(publish_lines rclcpp std_msgs sensor_msgs visualization_msgs)
target_link_libraries(publish_lines line_functions Lines)

install(
  TARGETS simple_lidar_reader save_single_measurement publish_lines
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
