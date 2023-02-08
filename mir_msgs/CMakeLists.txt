
cmake_minimum_required(VERSION 3.5)
project(mir_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/BatteryCurrents.msg"
  "msg/BMSData.msg"
  # "msg/BrakeState.msg"
  # "msg/ChargingState.msg"
  # "msg/Device.msg"
  # "msg/Devices.msg"
  # "msg/Encoders.msg"
  # "msg/Error.msg"
  # "msg/Event.msg"
  # "msg/Events.msg"
  # "msg/ExternalRobot.msg"
  # "msg/ExternalRobots.msg"
  # "msg/Gpio.msg"
  # "msg/GripperState.msg"
  # "msg/HeightState.msg"
  # "msg/HookExtendedStatus.msg"
  # "msg/HookStatus.msg"
  # "msg/IOs.msg"
  # "msg/JoystickVel.msg"
  # "msg/LocalMapStat.msg"
  # "msg/MirExtra.msg"
  # "msg/MirStatus.msg"
  # "msg/MissionCtrlCommand.msg"
  # "msg/MissionCtrlState.msg"
  # "msg/PalletLifterStatus.msg"
  # "msg/Path.msg"
  # "msg/Pendant.msg"
  # "msg/PlanSegment.msg"
  # "msg/PlanSegments.msg"
  # "msg/Pose2D.msg"
  # "msg/PowerBoardMotorStatus.msg"
  # "msg/Proximity.msg"
  # "msg/ResourcesState.msg"
  # "msg/ResourceState.msg"
  # "msg/RobotMode.msg"
  # "msg/RobotState.msg"
  # "msg/RobotStatus.msg"
  # "msg/SafetyStatus.msg"
  # "msg/Serial.msg"
  # "msg/StampedEncoders.msg"
  # "msg/Trolley.msg"
  # "msg/Twist2D.msg"
  # "msg/UserPrompt.msg"
  # "msg/WebPath.msg"
  # "msg/WorldMap.msg"
  # "msg/WorldModel.msg"
  "srv/ExecMission.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
