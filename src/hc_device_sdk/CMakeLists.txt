cmake_minimum_required(VERSION 3.8)
project(hc_device_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

add_library(hc_device_sdk SHARED
  src/mapping.cpp
  src/types.cpp
)
target_include_directories(hc_device_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(hc_device_sdk PRIVATE -Wall -Wextra -Wpedantic)

install(
  TARGETS hc_device_sdk
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY docs examples schema DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mapping test/test_mapping.cpp)
  target_link_libraries(test_mapping hc_device_sdk)

  ament_add_gtest(test_types test/test_types.cpp)
  target_link_libraries(test_types hc_device_sdk)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib)
ament_package()
