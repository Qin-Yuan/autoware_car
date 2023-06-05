cmake_minimum_required(VERSION 3.5)
project(autoware_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoiceCommand.msg"
  "msg/BtTts.msg"
  "msg/Position.msg"
  "msg/FloatList.msg"
  "action/ControlCommand.action"
  "srv/VoiceCommand.srv"
  "srv/ParamRequest.srv"
  DEPENDENCIES std_msgs action_msgs geometry_msgs
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
