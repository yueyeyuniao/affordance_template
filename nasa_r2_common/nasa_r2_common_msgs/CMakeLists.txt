cmake_minimum_required(VERSION 2.8.3)
project(nasa_r2_common_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation message_runtime std_msgs visualization_msgs geometry_msgs trajectory_msgs)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CommandStatus.msg
  ControllerJointSettings.msg
  ControllerPoseSettings.msg
  ForceControlAxisArray.msg
  ForceControlAxis.msg
  Gains.msg
  JointCapability.msg
  JointCommand.msg
  JointControlCalibrationMode.msg
  JointControlClearFaultMode.msg
  JointControlCoeffState.msg
  JointControlCommandMode.msg
  JointControlDataArray.msg
  JointControlData.msg
  JointControlMode.msg
  JointControl.msg
  JointStatusArray.msg
  JointStatus.msg
  JointTrajectoryReplan.msg
  LabeledControllerJointSettings.msg
  LabeledControllerPoseSettings.msg
  LabeledGains.msg
  LabeledGripperPositionCommand.msg
  LabeledJointControlDataArray.msg
  LabeledJointState.msg
  LabeledJointTrajectory.msg
  LabeledPoseTrajectory.msg
  LabeledTrajectoryMonitorFactors.msg
  MappedType.msg
  Modes.msg
  Parameter.msg
  PDMCStatus.msg
  PoseCommandArray.msg
  PoseCommand.msg
  PoseCommandParams.msg
  PoseCommandStatus.msg
  PoseState.msg
  PoseTrajectory.msg
  PoseTrajectoryPoint.msg
  PoseTrajectoryReplan.msg
  PoseTwistStamped.msg
  PowerState.msg
  PriorityArray.msg
  ReplanType.msg
  SetParameterRequest.msg
  SetParameterResponse.msg
  StringArray.msg
  SystemCore.msg
  SystemStatus.msg
  TorsoStatus.msg
  TrajectoryMonitorFactors.msg
  ValueType.msg
  VariantMapArray.msg
  VariantMap.msg
  WrenchMonitorLimit.msg
  WrenchState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ParseTableScene.srv
  Power.srv
  ResetTableScene.srv
  Servo.srv
  SetJointMode.srv
  SetTipName.srv
  TakeSnapshot.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_msgs
   CATKIN_DEPENDS std_msgs geometry_msgs visualization_msgs trajectory_msgs message_generation message_runtime
#  DEPENDS system_lib
)
