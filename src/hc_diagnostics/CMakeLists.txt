cmake_minimum_required(VERSION 3.16)
project(hc_diagnostics LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(hc_control_chain_monitor src/control_chain_monitor.cpp)
set_target_properties(hc_control_chain_monitor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hc_control_chain_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hc_control_chain_monitor PRIVATE -Wall -Wextra -Wpedantic)

add_library(hc_control_chain_diagnostics_component SHARED src/control_chain_node.cpp)
target_include_directories(hc_control_chain_diagnostics_component PRIVATE include)
target_link_libraries(hc_control_chain_diagnostics_component hc_control_chain_monitor)
target_compile_options(hc_control_chain_diagnostics_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_control_chain_diagnostics_component
  diagnostic_msgs hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_nodes(
  hc_control_chain_diagnostics_component "hc_diagnostics::ControlChainNode")

add_executable(control_chain_diagnostics_node src/main.cpp)
target_link_libraries(control_chain_diagnostics_node hc_control_chain_diagnostics_component)
target_compile_options(control_chain_diagnostics_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(control_chain_diagnostics_node rclcpp)

install(
  TARGETS hc_control_chain_monitor hc_control_chain_diagnostics_component
    control_chain_diagnostics_node
  EXPORT export_hc_diagnostics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_chain_monitor test/test_control_chain_monitor.cpp)
  target_link_libraries(test_control_chain_monitor hc_control_chain_monitor)
endif()

ament_export_targets(export_hc_diagnostics HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(diagnostic_msgs hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs)
ament_package()
