cmake_minimum_required(VERSION 3.16)
project(meister_serial_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED IMPORTED_TARGET libserial)

# firmware は PlatformIO 専用のツリーで ament パッケージではない。
# プロトコルの二重定義（Python と firmware の両方に実装を持っていた state）を
# 消したいため、ソースをインクルードするだけに留めて firmware は変更しない
# （AGENTS.md §0 で firmware は変更対象外）。
# 参照するのは Arduino 非依存の 2 ファイルだけ。
set(FIRMWARE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../firmware")

add_library(meister_proto STATIC
  ${FIRMWARE_DIR}/src/meister_protocol.cpp)
target_include_directories(meister_proto SYSTEM PUBLIC
  ${FIRMWARE_DIR}/include)

add_library(meister_serial_bridge_core STATIC
  src/bridge_core.cpp
  src/comm_check.cpp
  src/feedback.cpp
  src/feedback_hz_measure.cpp
  src/frames.cpp
  src/kinematics.cpp
  src/port_name.cpp
  src/serial_link.cpp
  src/stream_parser.cpp)
target_include_directories(meister_serial_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(meister_serial_bridge_core
  PUBLIC meister_proto
  PRIVATE PkgConfig::SERIAL)

add_executable(serial_bridge src/serial_bridge_node.cpp)
target_link_libraries(serial_bridge
  PRIVATE meister_serial_bridge_core rclcpp::rclcpp
          ${geometry_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(meister_comm_check src/comm_check_main.cpp)
target_link_libraries(meister_comm_check PRIVATE meister_serial_bridge_core)

add_executable(meister_hz_measure src/feedback_hz_measure_main.cpp)
target_link_libraries(meister_hz_measure PRIVATE meister_serial_bridge_core)

install(TARGETS serial_bridge meister_comm_check meister_hz_measure
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol meister_serial_bridge_core)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics meister_serial_bridge_core)
  ament_add_gtest(test_comm_check test/test_comm_check.cpp)
  target_link_libraries(test_comm_check meister_serial_bridge_core)
  ament_add_gtest(test_hz_measure test/test_hz_measure.cpp)
  target_link_libraries(test_hz_measure meister_serial_bridge_core)
  ament_add_gtest(test_bridge_core test/test_bridge_core.cpp)
  target_link_libraries(test_bridge_core meister_serial_bridge_core)
endif()

ament_package()
