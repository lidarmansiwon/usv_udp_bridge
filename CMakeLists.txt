cmake_minimum_required(VERSION 3.8)
project(usv_udp_bridge)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mk3_msgs REQUIRED)

include_directories(include)
install(
  DIRECTORY include/
  DESTINATION include
)

# UDP sender executable
add_executable(udp_sender src/sender.cpp)
ament_target_dependencies(udp_sender rclcpp std_msgs mk3_msgs)


# UDP receiver executable
add_executable(udp_receiver src/receiver.cpp)
ament_target_dependencies(udp_receiver rclcpp std_msgs mk3_msgs)

target_include_directories(udp_sender PUBLIC ${INCLUDE_DIRECTORIES})
target_include_directories(udp_receiver PUBLIC ${INCLUDE_DIRECTORIES})

# Install executables
install(TARGETS
  udp_sender
  udp_receiver
  DESTINATION lib/${PROJECT_NAME})

# Tests (Optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
