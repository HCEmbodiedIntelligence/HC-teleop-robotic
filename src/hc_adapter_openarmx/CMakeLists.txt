cmake_minimum_required(VERSION 3.16)
project(hc_adapter_openarmx LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(hc_openarmx_sim_core src/sim_joint_model.cpp)
target_include_directories(hc_openarmx_sim_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_openarmx_sim_core PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(hc_openarmx_sim_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(hc_openarmx_sim_component SHARED src/openarmx_sim_node.cpp)
target_include_directories(hc_openarmx_sim_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_openarmx_sim_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(
  hc_openarmx_sim_component
  hc_openarmx_sim_core
  kdl_parser::kdl_parser
  orocos-kdl
  urdf::urdf)
ament_target_dependencies(hc_openarmx_sim_component
  builtin_interfaces geometry_msgs hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(
  hc_openarmx_sim_component
  PLUGIN "hc_adapter_openarmx::OpenArmXSimNode"
  EXECUTABLE openarmx_sim_node)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS hc_openarmx_sim_core hc_openarmx_sim_component
  EXPORT export_hc_adapter_openarmx
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sim_joint_model test/test_sim_joint_model.cpp)
  if(TARGET test_sim_joint_model)
    target_link_libraries(test_sim_joint_model hc_openarmx_sim_core)
    target_compile_options(test_sim_joint_model PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_export_targets(export_hc_adapter_openarmx HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hc_teleop_interfaces)
ament_package()
