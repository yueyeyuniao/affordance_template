# Necessary configuration information for a robot end effector for use in RobotConfig msg

# the name of the end effector
string name

# the abstract template id that end-effector maps to
uint8 id

# offset position from abstract AT end-effector frame
geometry_msgs/Pose pose_offset

# tool offset pose to apply post end-effector frame
geometry_msgs/Pose tool_offset

