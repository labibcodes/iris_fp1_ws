cmake_minimum_required(VERSION 3.8)
project(master)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(main_controller src/main_controller.cpp)
ament_target_dependencies(main_controller rclcpp geometry_msgs std_msgs)

install(TARGETS
  main_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()