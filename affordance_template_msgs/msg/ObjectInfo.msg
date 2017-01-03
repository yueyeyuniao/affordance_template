# the name of the object frame
string object_name

# the pose of the object in a robot frame
geometry_msgs/PoseStamped object_pose

# shape type
string type

# if the object has a mesh
string mesh_resource

# shape size ([x,y,z] or [radius, 0, 0] or [radius, length, 0] -- for meshes/cubes, spheres, and cylinders respectively )
geometry_msgs/Vector3 size