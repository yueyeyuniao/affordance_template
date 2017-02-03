#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusMessage
from ihmc_msgs.msg import FootstepDataListMessage
from ihmc_msgs.msg import FootstepDataMessage

LEFT = 0
RIGHT = 1

def stepInPlace():
    msg = FootstepDataListMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5

    msg.footstep_data_list.append(createFootStepInPlace(LEFT))
    msg.footstep_data_list.append(createFootStepInPlace(RIGHT))
    msg.footstep_data_list.append(createFootStepInPlace(LEFT))
    msg.footstep_data_list.append(createFootStepInPlace(RIGHT))

    footStepListPublisher.publish(msg)
    print 'walking in place...'
    waitForFootsteps(len(msg.footstep_data_list))

def boxStep():
    msg = FootstepDataListMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5

    # walk forward starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.0, 0.0]))

    # walk left starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.2, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, 0.2, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.4, 0.4, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, 0.4, 0.0]))

    # walk back starting LEFT
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.4, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.4, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.4, 0.0]))

    # walk right starting RIGHT
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.2, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.2, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.0, 0.0]))

    footStepListPublisher.publish(msg)
    print 'box stepping...'
    waitForFootsteps(len(msg.footstep_data_list))

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

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    print 'finished set of steps'

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_box_step')

        footStepStatusSubscriber = rospy.Subscriber('/ihmc_ros/valkyrie/output/footstep_status', FootstepStatusMessage, recievedFootStepStatus)
        footStepListPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListMessage, queue_size=1)
        
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        # make sure the simulation is running otherwise wait
        if footStepListPublisher.get_num_connections() == 0:
            print 'waiting for subsciber...'
            while footStepListPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            stepInPlace()

        time.sleep(1)

        if not rospy.is_shutdown():
            boxStep()

    except rospy.ROSInterruptException:
        pass
