# Necessary configuration information for a robot that can be used to apply affordance templates to 

# the yaml file name of the robot configuration stored the affordance_template_library/robots/ directory
string filename

# the name of the robot
string name

# the name of the robot's configuration package that will enable the robot to be used by the ATs
string config_package

# the name of the robot's configuration file that will enable the robot to be used by the ATs
string config_file

# the name of the planner type to use for solving paths
string planner_type

# the robot's base frame id
string frame_id

# if the gripper/end-effector is to be controlled by a seperate action (not by a trajectory), this is the service name
GripperActionMap[] gripper_action

# offset position for where all ATs are initially placed in robot's base frame
geometry_msgs/Pose root_offset

# the list of  of robot end-effector group name (i.e., LeftHand) to the "abstract" AT end-effector ID 
EndEffectorConfig[] end_effectors

# the list of stored end-effector poses (i.e., "Left Hand Open") to be mapped the "abstract" AT pose IDs 
EndEffectorPoseData[] end_effector_pose_data

