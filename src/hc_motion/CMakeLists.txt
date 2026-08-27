cmake_minimum_required(VERSION 3.16)
project(hc_motion LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(hc_motion_core
  src/control_arbiter.cpp
  src/goal_monitor.cpp
  src/route_guard.cpp)
set_target_properties(hc_motion_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(hc_motion_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_options(hc_motion_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(hc_motion_router_component SHARED src/motion_router_node.cpp)
target_link_libraries(hc_motion_router_component hc_motion_core)
target_compile_options(hc_motion_router_component PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(hc_motion_router_component
  hc_teleop_interfaces rclcpp rclcpp_components)
rclcpp_components_register_nodes(
  hc_motion_router_component
  "hc_motion::MotionRouterNode")

add_executable(motion_router_node src/motion_router_main.cpp)
target_link_libraries(motion_router_node hc_motion_router_component)
target_compile_options(motion_router_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(motion_router_node rclcpp)

install(
  TARGETS hc_motion_core hc_motion_router_component motion_router_node
  EXPORT export_hc_motion
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_control_arbiter test_goal_monitor test_route_guard)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    if(TARGET ${test_name})
      target_link_libraries(${test_name} hc_motion_core)
      target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
  endforeach()
endif()

ament_export_targets(export_hc_motion HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(hc_teleop_interfaces)
ament_package()
