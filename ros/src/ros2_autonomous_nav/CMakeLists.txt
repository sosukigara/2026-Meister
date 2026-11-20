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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# 指標は ROS 非依存。host で gtest できる。
add_library(bench_metrics src/bench_metrics.cpp)
target_include_directories(bench_metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bench_metrics PUBLIC cxx_std_17)

# nav_world の地図を SDF ジオメトリから作る（SLAM 不要）。
add_library(map_generator src/map_generator.cpp)
target_include_directories(map_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(map_generator PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # Python 版の golden とバイト一致させる。壁の境界セルはちょうど半セルに
  # あり、FMA 融合の有無で 136 セルが変わる（-ffp-contract=off で一致を実測）。
  target_compile_options(map_generator PRIVATE -ffp-contract=off)
endif()

add_library(waypoint_tour src/waypoint_tour.cpp)
target_include_directories(waypoint_tour PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(waypoint_tour PUBLIC cxx_std_17)

add_library(nav_mode src/nav_mode.cpp)
target_include_directories(nav_mode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nav_mode PUBLIC cxx_std_17)

add_executable(nav_mode_manager src/nav_mode_manager_main.cpp)
target_link_libraries(nav_mode_manager nav_mode)
ament_target_dependencies(nav_mode_manager rclcpp nav2_msgs std_msgs std_srvs lifecycle_msgs)

add_executable(generate_map src/generate_map_main.cpp)
target_link_libraries(generate_map map_generator)

add_executable(waypoint_follower src/waypoint_follower_main.cpp)
target_link_libraries(waypoint_follower waypoint_tour)
ament_target_dependencies(waypoint_follower rclcpp rclcpp_action nav2_msgs)

add_library(bench_proc src/bench_proc.cpp)
target_include_directories(bench_proc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bench_proc PUBLIC cxx_std_17)

add_executable(bench_nav
  src/bench_nav_main.cpp
  src/bench_options.cpp
  src/costmap_view.cpp
  src/bench_report.cpp)
target_link_libraries(bench_nav bench_metrics bench_proc nlohmann_json::nlohmann_json)
ament_target_dependencies(bench_nav
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs tf2 tf2_ros)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS bench_nav generate_map waypoint_follower nav_mode_manager
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf worlds maps scripts
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bench_metrics test/test_bench_metrics.cpp)
  target_link_libraries(test_bench_metrics bench_metrics)

  ament_add_gtest(test_map_generator test/test_map_generator.cpp)
  target_link_libraries(test_map_generator map_generator)
  # Python 版が生成した地図とのバイト一致を、リポジトリ内で再現できる形で固定する。
  target_compile_definitions(test_map_generator PRIVATE
    MAP_GOLDEN_PATH="${CMAKE_CURRENT_SOURCE_DIR}/maps/nav_world.pgm")

  ament_add_gtest(test_waypoint_tour test/test_waypoint_tour.cpp)
  target_link_libraries(test_waypoint_tour waypoint_tour)

  ament_add_gtest(test_bench_proc test/test_bench_proc.cpp)
  target_link_libraries(test_bench_proc bench_proc)

  ament_add_gtest(test_nav_mode test/test_nav_mode.cpp)
  target_link_libraries(test_nav_mode nav_mode)

endif()

ament_package()
