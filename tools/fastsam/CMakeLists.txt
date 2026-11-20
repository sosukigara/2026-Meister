# FastSAM-x のライブ配信ノード。ビルドには参照実装のチェックアウトが要る
# （postprocess.cc をそのまま使う。自前実装へ置き換えるのは別作業）。
#
#   cmake -S tools/fastsam -B /tmp/fastsam_node/build \
#     -DFASTSAM_THIRD_PARTY=/tmp/fastsam_rknn/examples/rknn_fastSAM
#   cmake --build /tmp/fastsam_node/build -j4
cmake_minimum_required(VERSION 3.16)
project(fastsam_stream CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT FASTSAM_THIRD_PARTY)
  message(FATAL_ERROR "FASTSAM_THIRD_PARTY に参照実装の examples/rknn_fastSAM を指定する")
endif()
if(NOT EXISTS "${FASTSAM_THIRD_PARTY}/src/postprocess.cc")
  message(FATAL_ERROR "${FASTSAM_THIRD_PARTY} に postprocess.cc が無い")
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(fastsam_stream
  fastsam_stream.cc
  ${FASTSAM_THIRD_PARTY}/src/postprocess.cc)
target_include_directories(fastsam_stream PRIVATE
  ${FASTSAM_THIRD_PARTY}/include
  /usr/local/include/rknn)
target_link_libraries(fastsam_stream PRIVATE
  ${OpenCV_LIBS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  /usr/lib/librknnrt.so)
