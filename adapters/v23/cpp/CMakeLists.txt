cmake_minimum_required(VERSION 3.16)
project(hc_v23_ik_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Pin system Python 3.10 (ROS 2 Humble ABI) before pinocchio/eigenpy search.
set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "System Python3")
set(Python3_FIND_STRATEGY LOCATION)
set(Python3_ROOT_DIR "/usr")

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(v23_ik_core STATIC
  v23_ik_core.cpp
)
target_include_directories(v23_ik_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(v23_ik_core PUBLIC
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  yaml-cpp
)
target_compile_options(v23_ik_core PRIVATE -O3 -Wall -Wextra)

add_executable(controller_v2_3_ros2_cpp
  controller_v2_3_ros2_node.cpp
)
target_link_libraries(controller_v2_3_ros2_cpp PRIVATE
  v23_ik_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
)
target_compile_options(controller_v2_3_ros2_cpp PRIVATE -O3 -Wall -Wextra)

option(BUILD_PYBIND_MODULE "Build pybind11 verification/acceleration module" ON)
if(BUILD_PYBIND_MODULE)
  find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 REQUIRED)
  pybind11_add_module(_v23_ik_cpp MODULE pybind_v23_ik.cpp)
  target_link_libraries(_v23_ik_cpp PRIVATE v23_ik_core)
  target_compile_options(_v23_ik_cpp PRIVATE -O3 -Wall -Wextra)
  set_target_properties(_v23_ik_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../src"
  )
endif()
