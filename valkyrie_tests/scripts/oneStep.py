#!/usr/bin/env python

import time
import rospy
import tf
import tf2_ros
import numpy

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage

LEFT = 0
RIGHT = 1

def oneStep():
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 1.5
    msg.swing_time = 1.5

    # walk LEFT
    # msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.0, 0.4, 0.0, 0.0]))
    # return LEFt
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.1, 0.28, 0.0, 0.0]))
    # walk RIGHT
    # msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.0, -0.28, 0.0, 0.0]))
    # return RIGHT
    # msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.1, -0.3, 0.0, 0.0]))
    # walk to desk
    # msg.footstep_data_list.append(createFootStepOffset(LEFT, [0, 0.4, 0.0, 0.73]))#[-0.25, 0.5, 0.0, 0.73]
    msg.unique_id = 1

    footStepListPublisher.publish(msg)
    print 'one stepping...'

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = 'rightFoot'
    else:
        foot_frame = 'leftFoot'

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
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset[0:3])

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    (r, p, y) = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    y += offset[3]
    rotated_quat = tf.transformations.quaternion_from_euler(0, 0, y)

    footstep.orientation.x = rotated_quat[0]
    footstep.orientation.y = rotated_quat[1]
    footstep.orientation.z = rotated_quat[2]
    footstep.orientation.w = rotated_quat[3]

    return footstep

if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_one_step')

        footStepListPublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/footstep_list', FootstepDataListRosMessage, queue_size=1)
        
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(10) # 10hz
        time.sleep(1)

        if footStepListPublisher.get_num_connections() == 0:
            print 'waiting for subsciber...'
            while footStepListPublisher.get_num_connections() == 0:
                rate.sleep()

        if not rospy.is_shutdown():
            oneStep()
        
        time.sleep(1)

        # # make sure the simulation is running otherwise wait
        # if footStepListPublisher.get_num_connections() == 0:
        #     print 'waiting for subsciber...'
        #     while footStepListPublisher.get_num_connections() == 0:
        #         rate.sleep()

        # if not rospy.is_shutdown():
        #     stepInPlace()

        # time.sleep(1)

        # if not rospy.is_shutdown():
        #     boxStep()

    except rospy.ROSInterruptException:
        pass
