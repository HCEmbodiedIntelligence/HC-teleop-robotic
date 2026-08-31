cmake_minimum_required(VERSION 3.16)
project(hc_motion_backend_kdl LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(hc_joint_rate_limiter src/joint_rate_limiter.cpp)
target_compile_options(hc_joint_rate_limiter PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(hc_joint_rate_limiter PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hc_joint_rate_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(hc_kdl_ik_backend_component SHARED src/kdl_ik_backend_node.cpp)
target_compile_options(hc_kdl_ik_backend_component PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(hc_kdl_ik_backend_component PRIVATE include)
target_link_libraries(
  hc_kdl_ik_backend_component
  Eigen3::Eigen
  hc_joint_rate_limiter
  kdl_parser::kdl_parser
  orocos-kdl
  urdf::urdf)
ament_target_dependencies(hc_kdl_ik_backend_component
  builtin_interfaces geometry_msgs hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(
  hc_kdl_ik_backend_component
  PLUGIN "hc_motion_backend_kdl::KdlIkBackendNode"
  EXECUTABLE kdl_ik_backend_node)

install(
  TARGETS hc_joint_rate_limiter hc_kdl_ik_backend_component
  EXPORT export_hc_motion_backend_kdl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_rate_limiter test/test_joint_rate_limiter.cpp)
  target_link_libraries(test_joint_rate_limiter hc_joint_rate_limiter)
endif()

ament_export_targets(export_hc_motion_backend_kdl HAS_LIBRARY_TARGET)
ament_export_dependencies(hc_teleop_interfaces)
ament_package()
