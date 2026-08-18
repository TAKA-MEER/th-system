cmake_minimum_required(VERSION 3.8)
project(th_system_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── メッセージ（旧。WP-CLEAN-01 まで維持） ──────
  "msg/RobotMode.msg"
  "msg/FaultStatus.msg"
  "msg/WheelCommand.msg"
  "msg/WheelFeedback.msg"
  "msg/PanelArrival.msg"
  "msg/PersonStatus.msg"
  "msg/FollowStatus.msg"
  "msg/SearchStatus.msg"
  "msg/SummonStatus.msg"
  # ── メッセージ（新。WP-MSG-01） ──────────────
  "msg/SystemState.msg"
  "msg/StateEvent.msg"
  "msg/ActiveScreen.msg"
  "msg/LimiterStatus.msg"
  "msg/WaitClearStatus.msg"
  "msg/RouteInfo.msg"
  "msg/RouteList.msg"
  "msg/LinkQuality.msg"
  "msg/ParamsStatus.msg"
  "msg/PersonTargets.msg"
  "msg/Pin.msg"
  "msg/PinList.msg"
  "msg/RouteStatus.msg"
  "msg/MapSessionStatus.msg"
  "msg/CheckStatus.msg"
  "msg/CalibStatus.msg"
  # ── サービス（旧。WP-CLEAN-01 まで維持） ────────
  "srv/SetMode.srv"
  "srv/GoToPanel.srv"
  "srv/CompleteInspection.srv"
  "srv/SetTunableParams.srv"
  "srv/SaveTunableParams.srv"
  # ── サービス（新。WP-MSG-01） ──────────────
  "srv/UiTrigger.srv"
  "srv/SetFlag.srv"
  "srv/ListRoutes.srv"
  "srv/SaveRoute.srv"
  "srv/DeleteRoute.srv"
  "srv/TransferRoute.srv"
  "srv/OpenMapSession.srv"
  "srv/TwoPointPress.srv"
  "srv/RegisterPin.srv"
  "srv/EditPin.srv"
  "srv/DeclareHome.srv"
  "srv/EraseMapRegion.srv"
  "srv/RunCheck.srv"
  "srv/AnswerCheck.srv"
  "srv/StartCalib.srv"
  "srv/SubmitCalib.srv"
  "srv/ApplyCalib.srv"
  "srv/RollbackCalib.srv"
  "srv/GetParams.srv"
  "srv/SetParams.srv"
  DEPENDENCIES std_msgs geometry_msgs rcl_interfaces builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
