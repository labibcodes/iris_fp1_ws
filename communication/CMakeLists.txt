cmake_minimum_required(VERSION 3.8)
project(communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. WAJIB: Cari dependency package di sini terlebih dahulu
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Executable UDP Sender
add_executable(udp_sender src/udp_sender.cpp)
ament_target_dependencies(udp_sender rclcpp geometry_msgs)

# 3. Executable UDP Receiver
add_executable(udp_receiver src/udp_receiver.cpp)
ament_target_dependencies(udp_receiver rclcpp geometry_msgs)

# 4. Install target agar bisa dijalankan dengan ros2 run
install(TARGETS
  udp_sender
  udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()