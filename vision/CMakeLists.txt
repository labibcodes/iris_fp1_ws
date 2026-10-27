cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(line_detector src/line_detector.cpp)

# Tambahkan baris include directory ini agar header OpenCV & cv_bridge terdeteksi:
target_include_directories(line_detector PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(line_detector 
  rclcpp 
  sensor_msgs 
  std_msgs 
  cv_bridge 
  OpenCV
)

install(TARGETS
  line_detector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()