cmake_minimum_required(VERSION 3.16)
project(tienkung_policy_runner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bodyctrl_msgs REQUIRED)
find_package(tienkung_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime installation prefix")
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS
    "${ONNXRUNTIME_ROOT}/include"
    "${ONNXRUNTIME_ROOT}/include/onnxruntime/core/session"
)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib" "${ONNXRUNTIME_ROOT}/lib64"
)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime C++ was not found. Set ONNXRUNTIME_ROOT to its installation prefix.")
endif()

get_filename_component(ONNXRUNTIME_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  message(FATAL_ERROR
    "The bundled libfuncSPTrans.so targets Linux x86_64. Build this package on the deployment platform.")
endif()

set(FUNCSPTRANS_LIBRARY
  "${CMAKE_CURRENT_SOURCE_DIR}/native/lib/libfuncSPTrans.so.1.0.0")
if(NOT EXISTS "${FUNCSPTRANS_LIBRARY}")
  message(FATAL_ERROR "Missing vendor ankle transmission library: ${FUNCSPTRANS_LIBRARY}")
endif()
add_library(funcSPTrans SHARED IMPORTED GLOBAL)
set_target_properties(funcSPTrans PROPERTIES IMPORTED_LOCATION "${FUNCSPTRANS_LIBRARY}")

add_library(${PROJECT_NAME}_core SHARED
  src/fsm.cpp
  src/joint_transmission.cpp
  src/jsonl_logger.cpp
  src/motor_calibration.cpp
  src/observation_builder.cpp
  src/onnx_policy.cpp
  src/policy_runtime.cpp
  src/robot_config.cpp
  src/robot_io.cpp
  native/funcsptrans_c_api.cpp
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/native
)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC Eigen3::Eigen
  PRIVATE yaml-cpp ${ONNXRUNTIME_LIBRARY} funcSPTrans
)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(${PROJECT_NAME}_core PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/native/lib;${ONNXRUNTIME_LIBRARY_DIR}"
  INSTALL_RPATH "$ORIGIN;${ONNXRUNTIME_LIBRARY_DIR}"
)

add_library(${PROJECT_NAME}_ros SHARED src/policy_node.cpp)
target_include_directories(${PROJECT_NAME}_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# ament_target_dependencies() uses CMake's plain target_link_libraries form.
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp sensor_msgs bodyctrl_msgs tienkung_interfaces)
target_compile_options(${PROJECT_NAME}_ros PRIVATE -Wall -Wextra -Wpedantic)

add_executable(policy_runner_node src/policy_runner_main.cpp)
target_link_libraries(policy_runner_node ${PROJECT_NAME}_ros)
ament_target_dependencies(policy_runner_node rclcpp)

add_executable(dry_run_node src/dry_run_main.cpp)
target_link_libraries(dry_run_node ${PROJECT_NAME}_ros)
ament_target_dependencies(dry_run_node rclcpp)

add_executable(input_monitor_node src/input_monitor_main.cpp)
target_link_libraries(input_monitor_node ${PROJECT_NAME}_core)
ament_target_dependencies(input_monitor_node
  rclcpp sensor_msgs bodyctrl_msgs tienkung_interfaces)
target_compile_options(input_monitor_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS policy_runner_node dry_run_node input_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY native/lib/ DESTINATION lib
  FILES_MATCHING PATTERN "libfuncSPTrans.so*")

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_policy_core test/test_policy_core.cpp)
  if(TARGET test_policy_core)
    target_link_libraries(test_policy_core ${PROJECT_NAME}_core)
    target_compile_definitions(test_policy_core PRIVATE
      TEST_ROBOT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../tienkung_bringup/config/tg22_config.yaml"
      TEST_WALKER_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../tienkung_bringup/config/walker_config.yaml")
  endif()
endif()

ament_package()