cmake_minimum_required(VERSION 3.16)
project(umeonigiri_webnav CXX)

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
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(nlohmann_json REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# cpp-httplib は CMake config を同梱しない (0.14.3 時点) ので pkg-config で引く。
# libserial を引く umeonigiri_serial_bridge と同じ流儀。
find_package(PkgConfig REQUIRED)
pkg_check_modules(HTTPLIB REQUIRED IMPORTED_TARGET cpp-httplib)

add_library(umeonigiri_webnav_core SHARED
  src/http_app.cpp
  src/http_support.cpp
  src/map_cache.cpp
  src/map_store.cpp
  src/mode_worker.cpp
  src/map_listener.cpp
  src/nav_worker.cpp
  src/param_routes.cpp
  src/param_worker.cpp
  src/vision_cache.cpp
  src/vision_listener.cpp
  src/vision_routes.cpp
)
target_compile_features(umeonigiri_webnav_core PUBLIC cxx_std_17)
target_include_directories(umeonigiri_webnav_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(umeonigiri_webnav_core
  rclcpp rclcpp_action nav_msgs geometry_msgs tf2_msgs lifecycle_msgs nav2_msgs
  ament_index_cpp sensor_msgs vision_msgs std_srvs rcl_interfaces)
target_link_libraries(umeonigiri_webnav_core
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
  PkgConfig::HTTPLIB
)


# `ros2 run umeonigiri_webnav umeonigiri_webnav_server` と
# launch/umeonigiri_webnav.launch.py が入口。
add_executable(umeonigiri_webnav_server src/umeonigiri_webnav_server.cpp)
target_link_libraries(umeonigiri_webnav_server umeonigiri_webnav_core)

install(TARGETS umeonigiri_webnav_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS umeonigiri_webnav_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY webui DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_map_cache test/test_map_cache.cpp)
  target_link_libraries(test_map_cache umeonigiri_webnav_core)

  ament_add_gtest(test_map_store test/test_map_store.cpp)
  target_link_libraries(test_map_store umeonigiri_webnav_core)

  ament_add_gtest(test_map_listener test/test_map_listener.cpp)
  target_link_libraries(test_map_listener umeonigiri_webnav_core)

  ament_add_gtest(test_vision_cache test/test_vision_cache.cpp)
  target_link_libraries(test_vision_cache umeonigiri_webnav_core)

  ament_add_gtest(test_http_app test/test_http_app.cpp)
  target_link_libraries(test_http_app umeonigiri_webnav_core)
  # HTTP 面の test は install 前のソースツリーを配る。ROS を起動せずに済ませる。
  target_compile_definitions(test_http_app PRIVATE
    UMEONIGIRI_WEBUI_DIR="${CMAKE_CURRENT_SOURCE_DIR}/webui"
  )

  ament_add_gtest(test_vision_routes test/test_vision_routes.cpp)
  target_link_libraries(test_vision_routes umeonigiri_webnav_core)
  target_compile_definitions(test_vision_routes PRIVATE
    UMEONIGIRI_WEBUI_DIR="${CMAKE_CURRENT_SOURCE_DIR}/webui"
  )

  ament_add_gtest(test_param_routes test/test_param_routes.cpp)
  target_link_libraries(test_param_routes umeonigiri_webnav_core)
  target_compile_definitions(test_param_routes PRIVATE
    UMEONIGIRI_WEBUI_DIR="${CMAKE_CURRENT_SOURCE_DIR}/webui"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(umeonigiri_webnav_core)
ament_export_dependencies(rclcpp rclcpp_action nav_msgs sensor_msgs vision_msgs
  geometry_msgs tf2_msgs lifecycle_msgs nav2_msgs ament_index_cpp rcl_interfaces
  OpenCV nlohmann_json)

ament_package()
