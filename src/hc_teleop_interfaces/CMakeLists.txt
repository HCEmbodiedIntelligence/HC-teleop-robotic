cmake_minimum_required(VERSION 3.8)
project(hc_teleop_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControllerInput.msg"
  "msg/TrackedPose.msg"
  "msg/VrFrame.msg"
  "msg/MotionOptions.msg"
  "msg/CartesianTarget.msg"
  "msg/CartesianTargetArray.msg"
  "msg/CartesianState.msg"
  "msg/CartesianStateArray.msg"
  "msg/JointCommandCandidate.msg"
  "msg/JointCommand.msg"
  "msg/BaseCommandCandidate.msg"
  "msg/Status.msg"
  "msg/ComponentCapability.msg"
  "msg/RobotCapabilities.msg"
  "msg/SafetyState.msg"
  "msg/ControlLease.msg"
  "srv/SetEnabled.srv"
  "srv/ResetFault.srv"
  "srv/AcquireControl.srv"
  "action/Home.action"
  "action/MoveJ.action"
  "action/MoveL.action"
  "action/MoveP.action"
  "action/RecordDataset.action"
  "action/ReplayDataset.action"
  DEPENDENCIES
    action_msgs
    builtin_interfaces
    geometry_msgs
    sensor_msgs
    std_msgs
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_program(BASH_EXECUTABLE bash REQUIRED)

  ament_add_test(test_generated_interfaces
    COMMAND
      "${BASH_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/test_generated_interfaces.bash"
    APPEND_ENV
      "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
