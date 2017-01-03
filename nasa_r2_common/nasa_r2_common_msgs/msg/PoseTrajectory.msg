Header header
# length of nodes must match length of poses in each point
string[] nodes
# priorities must have nodes length or be omitted which defaults all to HIGH
PriorityArray[]  node_priorities
# refFrames must either have one item or be nodes length
string[] refFrames
PoseTrajectoryPoint[] points

