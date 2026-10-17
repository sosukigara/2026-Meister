cmake_minimum_required(VERSION 3.16)
project(meister_web_nav CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(nlohmann_json REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# cpp-httplib は CMake config を同梱しない (0.14.3 時点) ので pkg-config で引く。
# libserial を引く meister_serial_bridge と同じ流儀。
find_package(PkgConfig REQUIRED)
pkg_check_modules(HTTPLIB REQUIRED IMPORTED_TARGET cpp-httplib)

add_library(meister_web_nav_core SHARED
  src/http_app.cpp
  src/map_cache.cpp
  src/map_listener.cpp
  src/nav_worker.cpp
)
target_compile_features(meister_web_nav_core PUBLIC cxx_std_17)
target_include_directories(meister_web_nav_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(meister_web_nav_core
  rclcpp rclcpp_action nav_msgs geometry_msgs tf2_msgs lifecycle_msgs nav2_msgs
  ament_index_cpp)
target_link_libraries(meister_web_nav_core
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
  PkgConfig::HTTPLIB
)

# 実行ファイル名は Python 版のまま。launch/web_nav.launch.py と
# `ros2 run meister_web_nav web_nav_server` を変えない。
add_executable(web_nav_server src/web_nav_server.cpp)
target_link_libraries(web_nav_server meister_web_nav_core)

install(TARGETS meister_web_nav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS web_nav_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY webui DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_map_cache test/test_map_cache.cpp)
  target_link_libraries(test_map_cache meister_web_nav_core)

  ament_add_gtest(test_map_listener test/test_map_listener.cpp)
  target_link_libraries(test_map_listener meister_web_nav_core)

  ament_add_gtest(test_http_app test/test_http_app.cpp)
  target_link_libraries(test_http_app meister_web_nav_core)
  # HTTP 面の test は install 前のソースツリーを配る。ROS を起動せずに済ませる。
  target_compile_definitions(test_http_app PRIVATE
    MEISTER_WEBUI_DIR="${CMAKE_CURRENT_SOURCE_DIR}/webui"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(meister_web_nav_core)
ament_export_dependencies(rclcpp rclcpp_action nav_msgs geometry_msgs tf2_msgs
  lifecycle_msgs nav2_msgs ament_index_cpp OpenCV nlohmann_json)

ament_package()
