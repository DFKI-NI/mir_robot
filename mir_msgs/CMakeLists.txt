cmake_minimum_required(VERSION 3.5.1)
project(mir_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
    AngleMeasurment.msg
    BMSData.msg
    BatteryCurrents.msg
    BrakeState.msg
    ChargingState.msg
    Device.msg
    Devices.msg
    EncoderTestEntry.msg
    Encoders.msg
    Error.msg
    Event.msg
    Events.msg
    ExternalRobot.msg
    ExternalRobots.msg
    Gpio.msg
    GripperState.msg
    HeightState.msg
    HookData.msg
    HookExtendedStatus.msg
    HookStatus.msg
    IOs.msg
    JoystickVel.msg
    LocalMapStat.msg
    MirExtra.msg
    MirLocalPlannerPathTypes.msg
    MissionCtrlCommand.msg
    MissionCtrlState.msg
    MovingState.msg
    PalletLifterStatus.msg
    Path.msg
    Pendant.msg
    PlanSegment.msg
    PlanSegments.msg
    Pose2D.msg
    PowerBoardMotorStatus.msg
    PrecisionDockingStatus.msg
    Proximity.msg
    ResourceState.msg
    ResourcesAcquisition.msg
    ResourcesState.msg
    RobotMode.msg
    RobotState.msg
    RobotStatus.msg
    SafetyStatus.msg
    Serial.msg
    ServiceResponseHeader.msg
    SkidDetectionDiff.msg
    SkidDetectionStampedFloat.msg
    SoundEvent.msg
    StampedEncoders.msg
    TimeDebug.msg
    Trolley.msg
    Twist2D.msg
    UserPrompt.msg
    WebPath.msg
    WorldMap.msg
    WorldModel.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
)
