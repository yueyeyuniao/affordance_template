#!/usr/bin/env python

#
# A simple Python script showing the use of IHMC footstep,
# whole body, and arm messages. Uncomment the method in main
# that you want to run.
#


import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import WholeBodyTrajectoryRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

LEFT = 0
RIGHT = 1

LEFT_HOME = [-0.23, -1.06, 0.28, -1.4, 1.18, 0.02, 0.22]
RIGHT_HOME = [-0.25, 1.15, 0.3, 1.2, 1.08, -0.32, -0.19]

def wholeBody():
    #The message data structures
    msg = WholeBodyTrajectoryRosMessage()

    #Pelvis and chest position
    msg.pelvis_world_position.append(Vector3(0.0, 0.0, 0.65))
    msg.pelvis_linear_velocity.append(Vector3(0.0, 0.0, 0.0))
    msg.pelvis_angular_velocity.append(Vector3(0.0, 0.0, 0.0))
    msg.pelvis_world_orientation.append(Quaternion(0.0, 0.0, 0.0, 1.0))

    msg.chest_world_orientation.append(Quaternion(0.0, 0.0, 0.0, 1.0))
    msg.chest_angular_velocity.append(Vector3(0.0, 0.0, 0.0))

    #Arm trajectories
    msg.left_arm_trajectory = ArmTrajectoryRosMessage()
    msg.right_arm_trajectory = ArmTrajectoryRosMessage()
    msg.left_arm_trajectory.robot_side = ArmTrajectoryRosMessage.LEFT
    msg.right_arm_trajectory.robot_side = ArmTrajectoryRosMessage.RIGHT

    msg.left_arm_trajectory.trajectory_points = [createArmTrajectory(2.0, ZERO_VECTOR)]
    msg.right_arm_trajectory.trajectory_points = [createArmTrajectory(2.0, ZERO_VECTOR)]

    #Misc
    msg.time_at_waypoint = [1.0]
    msg.num_waypoints = 1
    msg.num_joints_per_arm = 7
    msg.unique_id = 1

    #Publish the message
    print 'publishing the whole body message'
    wholeBodyTrajectoryPublisher.publish(msg)

def sendRightArmTrajectory():
    msg = ArmTrajectoryRosMessage(robot_side = ArmTrajectoryRosMessage.RIGHT,
                joint_trajectory_messages = createArmTrajectory(2.0, 0.1, RIGHT_HOME),
                execution_mode = ArmTrajectoryRosMessage.OVERRIDE,
                unique_id = 1)

    print 'publishing right trajectory'
    if armTrajectoryPublisher.get_num_connections() == 0:
		while armTrajectoryPublisher.get_num_connections() ==0:
			print'waiting for publisher...'
			rate.sleep()
    armTrajectoryPublisher.publish(msg)

def sendLeftArmTrajectory():
    msg = ArmTrajectoryRosMessage(robot_side = ArmTrajectoryRosMessage.LEFT,
                joint_trajectory_messages = createArmTrajectory(2.0, 0.1, LEFT_HOME),
                execution_mode = ArmTrajectoryRosMessage.OVERRIDE,
                unique_id = 1)

    print 'publishing left trajectory'
    if armTrajectoryPublisher.get_num_connections() == 0:
		while armTrajectoryPublisher.get_num_connections() ==0:
			print'waiting for publisher...'
			rate.sleep()
    armTrajectoryPublisher.publish(msg)

def createArmTrajectory(time, velocity, positions):

    all_joints_trajectories = []
    for joint_angle in positions:
        individual_trajectory = TrajectoryPoint1DRosMessage(time = time, position = joint_angle, velocity = velocity)
        all_joints_trajectories.append(OneDoFJointTrajectoryRosMessage(trajectory_points = [individual_trajectory]))

    return all_joints_trajectories


if __name__ == '__main__':
    try:
        rospy.init_node('valkyrie_draw')

        #Set up in the right topics to publish and subscribe
        armTrajectoryPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/arm_trajectory', ArmTrajectoryRosMessage, queue_size=1)
        wholeBodyTrajectoryPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/whole_body_trajectory', WholeBodyTrajectoryRosMessage, queue_size=1)
 
        #Set up TF so we can place footsteps relative to the world frame       
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        if not rospy.is_shutdown():
            sendLeftArmTrajectory()
        time.sleep(1)       

        if not rospy.is_shutdown():
            sendRightArmTrajectory()
        time.sleep(1)

        # if not rospy.is_shutdown():
        #     wholeBody()
        # time.sleep(1)

    except rospy.ROSInterruptException:
        pass
