# Necessary status information for an affordance template 

# the name of the affordance template filename
string type

# the id of the template on the server, if there are multiple ones of the same type
uint8 id

# the name of the trajectory that is currently set
string trajectory_name

# The trajectory details (each element is for a different end_effector)
WaypointInfo[] waypoint_info

# The details if the objects (frames, and transformations)
ObjectInfo[] object_info
