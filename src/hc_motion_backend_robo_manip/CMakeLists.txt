cmake_minimum_required(VERSION 3.16)
project(hc_motion_backend_robo_manip LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# The pinned Motion Server source is connected to this workspace as a Git
# submodule. Its exported runtime target owns the private SDK ABI boundary.
find_package(humanoid_motion_server REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hc_teleop_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(robo_manip_backend_node src/robo_manip_backend_node.cpp)
target_compile_options(robo_manip_backend_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(robo_manip_backend_node
  humanoid_motion_server::humanoid_motion_runtime
  urdf::urdf
  yaml-cpp)
ament_target_dependencies(robo_manip_backend_node
  builtin_interfaces geometry_msgs hc_teleop_interfaces rclcpp sensor_msgs)

install(TARGETS robo_manip_backend_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
