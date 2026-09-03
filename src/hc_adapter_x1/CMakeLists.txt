cmake_minimum_required(VERSION 3.16)
project(hc_adapter_x1 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(hc_x1_sim_core src/sim_joint_model.cpp)
target_include_directories(hc_x1_sim_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_x1_sim_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(hc_x1_sim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(hc_x1_sim_component SHARED src/x1_sim_node.cpp)
target_include_directories(hc_x1_sim_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_x1_sim_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  hc_x1_sim_component
  hc_x1_sim_core
  kdl_parser::kdl_parser
  orocos-kdl
  urdf::urdf)
ament_target_dependencies(hc_x1_sim_component
  builtin_interfaces hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(
  hc_x1_sim_component
  PLUGIN "hc_adapter_x1::X1SimNode"
  EXECUTABLE x1_sim_node)

add_library(hc_x1_hardware_component SHARED src/x1_hardware_adapter_node.cpp)
target_include_directories(hc_x1_hardware_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_x1_hardware_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_x1_hardware_component
  builtin_interfaces hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(
  hc_x1_hardware_component
  PLUGIN "hc_adapter_x1::X1HardwareAdapterNode"
  EXECUTABLE x1_hardware_adapter_node)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS hc_x1_sim_core hc_x1_sim_component hc_x1_hardware_component
  EXPORT export_hc_adapter_x1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sim_joint_model test/test_sim_joint_model.cpp)
  if(TARGET test_sim_joint_model)
    target_link_libraries(test_sim_joint_model hc_x1_sim_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(hc_x1_sim_core hc_x1_sim_component hc_x1_hardware_component)
ament_export_targets(export_hc_adapter_x1)
ament_package()
