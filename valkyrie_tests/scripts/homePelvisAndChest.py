#!/usr/bin/env python

import rospy
import ihmc_msgs
from ihmc_msgs.msg import *
import time

if __name__ == '__main__':
	rospy.init_node ("move_pelvis_and_chest_to_home_position")
	rate = rospy.Rate(10) # 10hz

	goHomePub = rospy.Publisher("/ihmc_ros/valkyrie/control/go_home" , GoHomeRosMessage, queue_size = 1)

	#home pelvis message
	homePelvisMsg = GoHomeRosMessage(body_part = 2, robot_side = 0, trajectory_time = 6.0, unique_id = 1)
	if goHomePub.get_num_connections() == 0:
		print'waiting for publisher...'
		while goHomePub.get_num_connections() ==0:
			rate.sleep()
	goHomePub.publish(homePelvisMsg)
	time.sleep(5)

	#home chest message
	homeChestMsg = GoHomeRosMessage(body_part = 1, robot_side = 0, trajectory_time = 4.0, unique_id = 1)
	if goHomePub.get_num_connections() == 0:
		print'waiting for publisher...'
		while goHomePub.get_num_connections() ==0:
			rate.sleep()
	goHomePub.publish(homeChestMsg)
	time.sleep(5)

	#higher the pelvis and ready to walk
	pelvisHeightPub = rospy.Publisher("/ihmc_ros/valkyrie/control/pelvis_height_trajectory_adjusted_world", PelvisHeightTrajectoryRosMessage, queue_size = 1)
	pelvisHeightMsg = PelvisHeightTrajectoryRosMessage(trajectory_points = [
	TrajectoryPoint1DRosMessage( position = 1.05, velocity = 0, time = 4.0  )] 
	, unique_id = 1) #rospy.Time.now().secs)
	if pelvisHeightPub.get_num_connections() == 0:
		print'waiting for publisher...'
		while pelvisHeightPub.get_num_connections() ==0:
			rate.sleep()
	pelvisHeightPub.publish(pelvisHeightMsg)
	time.sleep(5)   