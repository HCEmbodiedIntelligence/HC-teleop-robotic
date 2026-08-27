cmake_minimum_required(VERSION 3.16)
project(hc_compat_bridge LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hc_vr_legacy_json src/vr_legacy_json.cpp)
set_target_properties(hc_vr_legacy_json PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hc_vr_legacy_json
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_vr_legacy_json PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_vr_legacy_json hc_teleop_interfaces)

add_library(hc_vr_legacy_bridge_component SHARED src/vr_legacy_bridge_node.cpp)
target_link_libraries(hc_vr_legacy_bridge_component hc_vr_legacy_json)
target_compile_options(hc_vr_legacy_bridge_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_vr_legacy_bridge_component
  hc_teleop_interfaces rclcpp rclcpp_components sensor_msgs std_msgs)
rclcpp_components_register_nodes(
  hc_vr_legacy_bridge_component
  "hc_compat_bridge::VrLegacyBridgeNode")

add_executable(vr_legacy_bridge_node src/vr_legacy_bridge_main.cpp)
target_link_libraries(vr_legacy_bridge_node hc_vr_legacy_bridge_component)
target_compile_options(vr_legacy_bridge_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(vr_legacy_bridge_node rclcpp)

install(
  TARGETS hc_vr_legacy_json hc_vr_legacy_bridge_component vr_legacy_bridge_node
  EXPORT export_hc_compat_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vr_legacy_json test/test_vr_legacy_json.cpp)
  if(TARGET test_vr_legacy_json)
    target_link_libraries(test_vr_legacy_json hc_vr_legacy_json)
    target_compile_options(test_vr_legacy_json PRIVATE -Wall -Wextra -Wpedantic)
    ament_target_dependencies(test_vr_legacy_json hc_teleop_interfaces)
  endif()
endif()

ament_export_targets(export_hc_compat_bridge HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hc_teleop_interfaces)
ament_package()
