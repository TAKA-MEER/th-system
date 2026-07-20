cmake_minimum_required(VERSION 3.8)
project(th_system_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── メッセージ ──────────────────────────────
  "msg/RobotMode.msg"
  "msg/FaultStatus.msg"
  "msg/WheelCommand.msg"
  "msg/WheelFeedback.msg"
  "msg/PanelArrival.msg"
  "msg/PersonStatus.msg"
  # ── サービス ────────────────────────────────
  "srv/SetMode.srv"
  "srv/GoToPanel.srv"
  "srv/CompleteInspection.srv"
  "srv/SetTunableParams.srv"
  "srv/SaveTunableParams.srv"
  DEPENDENCIES std_msgs geometry_msgs rcl_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
