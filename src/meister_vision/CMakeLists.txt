cmake_minimum_required(VERSION 3.16)
project(meister_vision CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio dnn)
find_package(onnxruntime_vendor REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(meister_vision_core SHARED
  src/letterbox.cpp
  src/nms.cpp
  src/onnx_session.cpp
  src/yolo_detector.cpp
  src/hand_landmark_detector.cpp
  src/image_util.cpp
  src/draw.cpp
)
target_compile_definitions(meister_vision_core PRIVATE
  MEISTER_VISION_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_compile_features(meister_vision_core PUBLIC cxx_std_17)
target_include_directories(meister_vision_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(meister_vision_core
  rclcpp sensor_msgs geometry_msgs vision_msgs ament_index_cpp)
target_link_libraries(meister_vision_core
  ${OpenCV_LIBS}
  onnxruntime::onnxruntime
)

set(VISION_EXECUTABLES
  detection_node
  hand_landmarks_node
  pc_camera
)
# 実行ファイル名とソース名が違うのは pc_camera だけ（ソースは Node 付き）。
set(VISION_SOURCES
  detection_node.cpp
  hand_landmarks_node.cpp
  pc_camera_node.cpp
)
list(LENGTH VISION_EXECUTABLES exe_count)
math(EXPR last "${exe_count} - 1")
foreach(i RANGE ${last})
  list(GET VISION_EXECUTABLES ${i} exe)
  list(GET VISION_SOURCES ${i} src)
  add_executable(${exe} src/${src})
  target_link_libraries(${exe} meister_vision_core)
endforeach()

install(TARGETS meister_vision_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS detection_node hand_landmarks_node pc_camera
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

# モデル DL は Python のまま残す（ネットワーク取得だけなので C++ にしても
# 得られるものが libcurl 依存の追加だけ）。実行ファイル名は download_model で
# 維持し、`ros2 run meister_vision download_model` を壊さない。
install(PROGRAMS scripts/download_model.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_letterbox test/test_letterbox.cpp)
  target_link_libraries(test_letterbox meister_vision_core)

  ament_add_gtest(test_postprocess test/test_postprocess.cpp)
  target_link_libraries(test_postprocess meister_vision_core)

  ament_add_gtest(test_hand_landmarks test/test_hand_landmarks.cpp)
  target_link_libraries(test_hand_landmarks meister_vision_core)

  ament_add_gtest(test_image_util test/test_image_util.cpp)
  target_link_libraries(test_image_util meister_vision_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(meister_vision_core)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs vision_msgs
  ament_index_cpp OpenCV onnxruntime_vendor)

ament_package()