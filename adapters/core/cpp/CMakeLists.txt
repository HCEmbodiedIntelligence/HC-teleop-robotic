cmake_minimum_required(VERSION 3.16)
project(hc_adapters_core_cpp LANGUAGES CXX)

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
find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET REQUIRED bullet-float64)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

# 1. C++17 Eigen-accelerated 100 Hz teleop pose/quaternion/slew math
pybind11_add_module(_arm_teleop_math_cpp MODULE arm_teleop_math_cpp.cpp)
target_include_directories(_arm_teleop_math_cpp PRIVATE ${EIGEN3_INCLUDE_DIR})
target_compile_options(_arm_teleop_math_cpp PRIVATE -O3 -Wall -Wextra)
set_target_properties(_arm_teleop_math_cpp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

# 2. C++17 Pinocchio + Bullet3 (float64) native self-collision guard
pybind11_add_module(_collision_guard_cpp MODULE collision_guard_cpp.cpp)
target_compile_definitions(_collision_guard_cpp PRIVATE BT_USE_DOUBLE_PRECISION)
target_include_directories(_collision_guard_cpp PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
)
target_link_libraries(_collision_guard_cpp PRIVATE
  pinocchio::pinocchio_default
  pinocchio::pinocchio_parsers
  pinocchio::pinocchio_collision
  ${urdfdom_LIBRARIES}
  BulletCollision-float64
  LinearMath-float64
)
target_compile_options(_collision_guard_cpp PRIVATE -O3 -Wall -Wextra)
set_target_properties(_collision_guard_cpp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/.."
)

