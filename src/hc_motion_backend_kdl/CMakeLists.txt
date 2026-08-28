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

add_library(hc_kdl_ik_backend_component SHARED src/kdl_ik_backend_node.cpp)
target_compile_options(hc_kdl_ik_backend_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  hc_kdl_ik_backend_component
  Eigen3::Eigen
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
  TARGETS hc_kdl_ik_backend_component
  EXPORT export_hc_motion_backend_kdl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_hc_motion_backend_kdl HAS_LIBRARY_TARGET)
ament_export_dependencies(hc_teleop_interfaces)
ament_package()
