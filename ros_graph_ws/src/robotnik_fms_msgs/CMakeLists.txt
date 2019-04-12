cmake_minimum_required(VERSION 2.8.3)
project(robotnik_fms_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  robotnik_msgs
  geometry_msgs
  robot_local_control_msgs
  message_generation
  procedures_msgs
)

add_message_files(
   FILES
   RobotStatus.msg
   MissionsStatusString.msg
   Mission.msg
   CurrentMissions.msg
   TaskData.msg
   State.msg
   NodeInfo.msg
   NodesInfo.msg
   NodesID.msg
   PlannerRobotStatus.msg
   PlannerRobotStatusArray.msg
   DoorStatus.msg
   DoorStatusArray.msg
   PresenceSensor.msg
   PresenceSensorArray.msg
   CellSet.msg
   CellSetArray.msg
   MetaPlannerStatus.msg
   MetaPlannerStatusArray.msg
)

add_service_files(
  FILES
  RobotNodesTasks.srv
  GetRobotStatus.srv
#  robot_go_to.srv
  GetMissions.srv
  UpdateMission.srv
  InsertMission.srv
## services for fms_backend_interface
  RobotGoToAndWait.srv
  RobotContinue.srv
  RobotPickAndPlace.srv
  RobotBatteryCharge.srv
  RobotCancel.srv
  GetRobotsStatus.srv
#####
  Doors.srv
  SetLights.srv
  GetRoute.srv
  ReloadGraph.srv
  GetNodeInfo.srv
  BlockNode.srv
  GetBlockedNode.srv
  SetPhotocell.srv
  ControlElevator.srv
  GenerateMission.srv
  InitializeCart.srv
  PushButton.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  robotnik_msgs
  robot_local_control_msgs
  geometry_msgs
  procedures_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES procedures_test_msgs
 CATKIN_DEPENDS message_runtime
                std_msgs
                robotnik_msgs
                robot_local_control_msgs
                geometry_msgs
                procedures_msgs
#  DEPENDS system_lib
)
