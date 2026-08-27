cmake_minimum_required(VERSION 3.16)
project(hc_vr_gateway LANGUAGES CXX)

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
find_package(std_msgs REQUIRED)

add_library(hc_vr_codec src/codec.cpp)
set_target_properties(hc_vr_codec PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hc_vr_codec
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_vr_codec PRIVATE -Wall -Wextra -Wpedantic)

add_library(hc_vr_gateway_component SHARED src/vr_gateway_node.cpp)
target_link_libraries(hc_vr_gateway_component hc_vr_codec)
target_compile_options(hc_vr_gateway_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_vr_gateway_component
  builtin_interfaces
  hc_teleop_interfaces
  rclcpp
  rclcpp_components
  std_msgs)
rclcpp_components_register_nodes(
  hc_vr_gateway_component
  "hc_vr_gateway::VrGatewayNode")

add_executable(hc_vr_gateway_node src/main.cpp)
target_link_libraries(hc_vr_gateway_node hc_vr_gateway_component)
target_compile_options(hc_vr_gateway_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_vr_gateway_node rclcpp)

install(
  TARGETS hc_vr_codec hc_vr_gateway_component hc_vr_gateway_node
  EXPORT export_hc_vr_gateway
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_codec test/test_codec.cpp)
  if(TARGET test_codec)
    target_link_libraries(test_codec hc_vr_codec)
    target_compile_options(test_codec PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_export_targets(export_hc_vr_gateway HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_components hc_teleop_interfaces std_msgs)
ament_package()
