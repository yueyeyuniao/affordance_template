# joint name/id maps
JointNameMap joint_map

# group name
string group_name

# group type
string group_type

# active flag
bool is_active

# cartesian planning flag
bool compute_cartesian_plans

# group ID
int8 group_id

# group control frame
string control_frame

# group planning frame
string planning_frame

# joint mask
JointMask joint_mask

# plan on move
bool plan_on_move

# execute on plan
bool execute_on_plan

# stored pose name to execute/plan-for
string stored_pose

# stored pose names
string[] stored_pose_list

# goal pose
geometry_msgs/PoseStamped goal_pose

# the comdintioning metric
string conditioning_metric

# available conditioning metrics
string[] conditioning_metrics

# the task compatibility vector
geometry_msgs/Pose compatibility_vector

# the different tolerance modes (applicable to all groups)
ToleranceInfo[] tolerances

# current tolerance settings for each group
ToleranceInfo[] tolerance_settings
