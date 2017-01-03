# Waypoint Information for use in Affordance Templates

# the abstract ID of the end effector
uint8 id

# the name of the end effector
string end_effector_name

# the number of waypoints for this the end effector
uint8 num_waypoints

# the current index of the end effector in a trajectory
int8 waypoint_index

# the index of the waypoint that was last planned to
int8 waypoint_plan_index

# the plan status
bool plan_valid

# the execution status
bool execution_valid

# waypoint info
geometry_msgs/PoseStamped[] waypoint_poses

# end_effector pose names
string[] end_effector_pose_names

# end_effector control frame
string control_frame

# waypoint is in compact_view
bool[] compact_view
