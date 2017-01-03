cmake_minimum_required(VERSION 2.8.3)
project(affordance_template_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  trajectory_msgs
  diagnostic_msgs
  message_generation
  actionlib_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  EndEffectorConfig.msg
  EndEffectorPoseData.msg
  RobotConfig.msg
  WaypointInfo.msg
  ObjectInfo.msg
  WaypointTrajectory.msg
  AffordanceTemplateConfig.msg
  AffordanceTemplateStatus.msg
  RecognitionObjectConfig.msg
  ScaleDisplayObjectInfo.msg
  GripperActionMap.msg
  DisplayObjectInfo.msg
  WaypointViewMode.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetRobotConfigInfo.srv
  GetAffordanceTemplateConfigInfo.srv
  GetRecognitionObjectConfigInfo.srv
  GetRunningAffordanceTemplates.srv
  GetAffordanceTemplateStatus.srv
  GetAffordanceTemplateServerStatus.srv
  LoadRobotConfig.srv
  AddAffordanceTemplate.srv
  DeleteAffordanceTemplate.srv
  AddRecognitionObject.srv
  DeleteRecognitionObject.srv
  AffordanceTemplatePlanCommand.srv
  AffordanceTemplateExecuteCommand.srv
  SaveAffordanceTemplate.srv
  AddAffordanceTemplateTrajectory.srv
  ScaleDisplayObject.srv
  SetAffordanceTemplateTrajectory.srv
  SetAffordanceTemplatePose.srv
  GetObjectPose.srv
  SetObjectPose.srv
  SetWaypointViewModes.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Plan.action Execute.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  trajectory_msgs
  diagnostic_msgs
  actionlib_msgs
)

catkin_package( CATKIN_DEPENDS message_runtime geometry_msgs std_msgs trajectory_msgs diagnostic_msgs actionlib_msgs )

include_directories( ${catkin_INCLUDE_DIRS} )