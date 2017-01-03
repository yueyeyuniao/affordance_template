# Necessary configuration information for an recognition object 

# the name of the recognition object, should match name stored in the corresponding json file in affordance_template_library/recognition_objects/ directory
string type

# the name of the launch file to run to start the recognition object node(s)
string launch_file

# the package where that launch file resides
string package

# the name of the image to be displayed in the RO panel in rviz in  affordance_template_library/images/
string image_path

# the name of the marker topic to subscribe to
string marker_topic
