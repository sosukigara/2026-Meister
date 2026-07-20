cmake_minimum_required(VERSION 3.8)
project(perception)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  launch/perception.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
