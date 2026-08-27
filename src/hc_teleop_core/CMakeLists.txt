cmake_minimum_required(VERSION 3.8)
project(hc_teleop_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(hc_command_arbiter_core SHARED src/command_arbiter.cpp)
target_include_directories(hc_command_arbiter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hc_command_arbiter_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(hc_vr_mapper_core SHARED src/vr_mapper.cpp)
target_include_directories(hc_vr_mapper_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hc_vr_mapper_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(hc_command_arbiter_component SHARED src/command_arbiter_node.cpp)
target_include_directories(hc_command_arbiter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hc_command_arbiter_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hc_command_arbiter_component hc_command_arbiter_core)
ament_target_dependencies(hc_command_arbiter_component
  builtin_interfaces hc_teleop_interfaces rclcpp rclcpp_components)
rclcpp_components_register_nodes(
  hc_command_arbiter_component "hc_teleop_core::CommandArbiterNode")

add_library(hc_vr_mapper_component SHARED src/vr_mapper_node.cpp)
target_include_directories(hc_vr_mapper_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hc_vr_mapper_component PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(hc_vr_mapper_component hc_vr_mapper_core)
ament_target_dependencies(hc_vr_mapper_component
  builtin_interfaces hc_teleop_interfaces rclcpp rclcpp_components)
rclcpp_components_register_nodes(
  hc_vr_mapper_component "hc_teleop_core::VrMapperNode")

add_library(hc_auto_lease_component SHARED src/auto_lease_node.cpp)
target_include_directories(hc_auto_lease_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(hc_auto_lease_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_auto_lease_component
  builtin_interfaces hc_teleop_interfaces rclcpp rclcpp_components)
rclcpp_components_register_node(
  hc_auto_lease_component
  PLUGIN "hc_teleop_core::AutoLeaseNode"
  EXECUTABLE auto_lease_node)

add_executable(command_arbiter_node src/command_arbiter_main.cpp)
target_link_libraries(command_arbiter_node hc_command_arbiter_component)
ament_target_dependencies(command_arbiter_node rclcpp)

add_executable(vr_mapper_node src/vr_mapper_main.cpp)
target_link_libraries(vr_mapper_node hc_vr_mapper_component)
ament_target_dependencies(vr_mapper_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  hc_command_arbiter_core
  hc_command_arbiter_component
  hc_vr_mapper_core
  hc_vr_mapper_component
  hc_auto_lease_component
  command_arbiter_node
  vr_mapper_node
  auto_lease_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_arbiter test/test_command_arbiter.cpp)
  if(TARGET test_command_arbiter)
    target_link_libraries(test_command_arbiter hc_command_arbiter_core)
  endif()
  ament_add_gtest(test_vr_mapper test/test_vr_mapper.cpp)
  if(TARGET test_vr_mapper)
    target_link_libraries(test_vr_mapper hc_vr_mapper_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(hc_command_arbiter_core hc_vr_mapper_core)
ament_export_dependencies(hc_teleop_interfaces)
ament_package()
