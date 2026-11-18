cmake_minimum_required(VERSION 3.16)
project(ros2_autonomous_nav CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

# 指標は ROS 非依存。host で gtest できる。
add_library(bench_metrics src/bench_metrics.cpp)
target_include_directories(bench_metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bench_metrics PUBLIC cxx_std_17)

add_executable(bench_nav src/bench_nav_main.cpp)
target_link_libraries(bench_nav bench_metrics nlohmann_json::nlohmann_json)
ament_target_dependencies(bench_nav
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs tf2 tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS bench_nav DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf worlds maps scripts
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bench_metrics test/test_bench_metrics.cpp)
  target_link_libraries(test_bench_metrics bench_metrics)
endif()

ament_package()
