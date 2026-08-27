cmake_minimum_required(VERSION 3.8)
project(hc_robot_openarmx)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE.openarmx_description DESTINATION share/${PROJECT_NAME})

ament_package()
