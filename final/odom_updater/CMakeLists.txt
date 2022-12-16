cmake_minimum_required(VERSION 3.8)
project(odom_updater)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include/odom_updater)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



#--------BroadcastPose class : start ---------
add_executable(odom_updater app/main.cpp src/odom_updater.cpp)
ament_target_dependencies(odom_updater rclcpp tf2_ros nav_msgs ament_cmake geometry_msgs)

install(TARGETS odom_updater DESTINATION lib/${PROJECT_NAME})
#--------BroadcastPose class : end -----------


ament_package()
