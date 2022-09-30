cmake_minimum_required(VERSION 3.5)
project(planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_interfaces REQUIRED)
include_directories(include)
add_executable(pl_node
  src/ntzx_conf_app.cpp
  src/pl_node.cpp
  src/ntzx_download_info.cpp
  src/ntzx_file_drv.cpp
  src/ntzx_fuse_data.cpp
  src/ntzx_lidar_leishen.cpp
  src/ntzx_file_drv.cpp
  src/ntzx_log_app.cpp
  src/ntzx_mc.cpp
  src/ntzx_qianxun_inv.cpp
  src/ntzx_timer_drv.cpp
  src/ntzx_tools.cpp
  src/ntzx_usart_drv.cpp
  ) 
ament_target_dependencies(pl_node rclcpp std_msgs message_interfaces)

install(TARGETS
pl_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
