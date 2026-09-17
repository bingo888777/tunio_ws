cmake_minimum_required(VERSION 3.8)
project(tunio_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StringResult.msg"
  "msg/TuningState.msg"
  "action/MeasurePitch.action"
  "action/PlayString.action"
  "action/AdjustPeg.action"
  "action/TransferGuitar.action"
  "srv/StartTuning.srv"
  "srv/ControlCommand.srv"
  "srv/GetHistory.srv"
  "srv/GetSettings.srv"
  "srv/SetSettings.srv"
  DEPENDENCIES std_msgs
)

ament_package()
