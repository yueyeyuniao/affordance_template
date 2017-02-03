#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from ihmc_msgs.msg import FootstepStatusMessage
from ihmc_msgs.msg import HandPosePacketMessage
from ihmc_msgs.msg import FootstepDataListMessage
from ihmc_msgs.msg import FootstepDataMessage


LEFT = 0
RIGHT = 1
isStepping = False
stepCounter = 1
stepsize = 0.3

def sendStepTarget():
	msg = PoseStamped()
	msg.pose.position.x = 0
	msg.pose.position.y = stepsize
	msg.pose.position.z = 0 # doesn't matter
	msg.pose.orientation.x = 0 # turn 90 degree
	msg.pose.orientation.y = 0
	msg.pose.orientation.z = 0
	msg.pose.orientation.w = 1

	footTargetPublisher.publish(msg)
	print 'walking to place...'

def recievedFootStepStatus(msg):
    
    if msg.status == 1:
        stepCounter += 1
        isStepping = True

def boxStep():
    msg = FootstepDataListMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5

    # walk forward starting LEFT 
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.05, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, -0.05, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.05, 0.0]))

    # walk left starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, -0.1, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, -0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, -0.15, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, -0.05, 0.0]))

    # # walk back starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.2, -0.15, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, -0.05, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -0.15, 0.0]))

    # # walk right starting RIGHT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, -0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -0.1, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.05, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -0.05, 0.0]))

    footStepListPublisher.publish(msg)
    print 'box stepping...'

# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset):
    footstep = createFootStepInPlace(stepSide)

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = 'leftFoot'
    else:
        foot_frame = 'rightFoot'

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

def sendPoseTarget(traj_time):
    rate = rospy.Rate(10) # 10hz
    msg = HandPosePacketMessage()
    msg.robot_side = msg.LEFT
    msg.data_type = msg.HAND_POSE
    msg.reference_frame = msg.CHEST
    msg.position.x = 0.4
    msg.position.y = 0.4
    msg.position.z = -0.1
    msg.orientation.w = 1
    msg.orientation.x = 0
    msg.orientation.y = 0
    msg.orientation.z = 0
    msg.trajectory_time = traj_time
    msg.control_orientation = True
    msg.unique_id += stepCounter

    if handPosePublisher.get_num_connections() == 0:
        print'waiting for publisher...'
        while handPosePublisher.get_num_connections() ==0:
            rate.sleep()

    if not rospy.is_shutdown():
        handPosePublisher.publish(msg)
    
    print 'set the pose...'


if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')

        footStepStatusSubscriber = rospy.Subscriber('/ihmc_ros/valkyrie/output/footstep_status', FootstepStatusMessage, recievedFootStepStatus)
        footTargetPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/endpoint_footstep_generator', PoseStamped, queue_size=1)
        footStepListPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListMessage, queue_size=1)
        handPosePublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/hand_pose', HandPosePacketMessage, queue_size = 1)
        
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # send pose command
        sendPoseTarget(3)
        time.sleep(5)

        # sendPoseTarget(0.01)
        # # make sure the simulation is running otherwise wait
        # if footStepListPublisher.get_num_connections() == 0:
        #     print 'waiting for publisher...'
        #     while footStepListPublisher.get_num_connections() == 0:
        #         rate.sleep()

        # if not rospy.is_shutdown():
        #     boxStep()
        #     rate.sleep()

        # while True:
        #     sendPoseTarget(0.01)
        #     rate.sleep()


    except rospy.ROSInterruptException:
        pass
