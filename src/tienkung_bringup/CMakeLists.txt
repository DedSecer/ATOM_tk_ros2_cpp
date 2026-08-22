cmake_minimum_required(VERSION 3.8)
project(tienkung_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(PROGRAMS check_walker_policy.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()