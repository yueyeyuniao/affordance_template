# plan feedtack types
uint8 PLAN_UNKNOWN = 0
uint8 NO_PLAN_FOUND = 1
uint8 PLAN_FOUND = 2

# the name of the end effector
string end_effector_name

# the waypoint id that the ee is currently at
int8 waypoint_idx

# the waypoint id that the ee is currently at
int8 final_waypoint_idx

# the plan status
uint8 plan_status

