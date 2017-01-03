# Necessary configuration information for an affordance template 

# the name of the affordance template filename
string filename

# the name of the affordance template, should match name stored in the corresponding json file in affordance_template_library/templates/ directory
string type

# the id of the template on the server, if there are multiple ones of the same type
uint8 id

# the name of the image to be displayed in the AT panel in rviz in  affordance_template_library/images/
string image_path

# The trajectory details
WaypointTrajectory[] trajectory_info

# The display objects
string[] display_objects
