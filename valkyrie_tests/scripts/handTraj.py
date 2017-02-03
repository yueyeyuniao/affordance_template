#!/usr/bin/env python

import time
import rospy
import tf
from tf.msg import tfMessage
import tf2_ros                #draw links in rviz tf2
import numpy

from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point


class valk():

  def __init__(self,*kwargs):

    self.HP = Pose()
    self.currentPose = Pose()
    # initialize node
    rospy.init_node("valk_move")
    # add tf listener
    tf_buffer = tf2_ros.Buffer() 
    tf2_ros.TransformListener(tf_buffer)
    self.listener = tf_buffer
    time.sleep(1)

    self.handside = 'leftPalm'

    try:
      pose = self.listener.lookup_transform('world', self.handside, rospy.Time(0))
    except tf.LookupException as ex:
      print ex

    print "currentpose:",pose

    # set hand center offset with respect to wrist frame
    isLeftHand = -1;
    if self.handside == 'leftPalm':
       isLeftHand = 1; 
    offset = [0.005, isLeftHand*0.1, 0.0]
    rot = tf.transformations.quaternion_matrix([pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w])
    rot_offset = tf.transformations.quaternion_matrix([0,0,-0.02618,0.99966])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)
    print "offset",transformedOffset
    print "rot_old:",pose.transform.rotation
    rot = numpy.dot(rot_offset,rot)
    rotation_array = tf.transformations.quaternion_from_matrix(rot)
    print "quat_new:",rotation_array

    self.HP.position.x = pose.transform.translation.x + transformedOffset[0]
    self.HP.position.y = pose.transform.translation.y + transformedOffset[1]
    self.HP.position.z = pose.transform.translation.z + transformedOffset[2]
    self.HP.orientation.x = rotation_array[0]
    self.HP.orientation.y = rotation_array[1]
    self.HP.orientation.z = rotation_array[2]
    self.HP.orientation.w = rotation_array[3]
    print "home",self.HP

    # setup publisher for handTrajectory*
    self.hand_pub = rospy.Publisher('/ihmc_ros/valkyrie/control/hand_trajectory', HandTrajectoryRosMessage, queue_size=30)
    self.arm_pub = rospy.Publisher("/ihmc_ros/valkyrie/control/arm_trajectory", ArmTrajectoryRosMessage, queue_size=1)

    rate = rospy.Rate(10)


  def moveArmHome(self):
    print "moving arm to home position"
    rs = 1
    if self.handside == 'leftPalm':
      rs = 0
    ex = 0
    ref = 1
    #position = Vector3(0.3,0.20,1.3)
    # orientation = Quaternion(0,0,0,1.0)
    # linvel = Vector3(0.1,0.1,0.1)
    # angvel = Vector3(0.1,0.1,0.1)
    # create trajectory message
    traj = SE3TrajectoryPointRosMessage(time=3.0, position=self.HP.position, orientation=self.HP.orientation)
    msg = HandTrajectoryRosMessage(robot_side=rs, taskspace_trajectory_points=[traj], execution_mode=ex, base_for_control=1, unique_id=1)
    # publish message
    self.hand_pub.publish(msg)
  
  def moveArmRelative(self,position):
    rs = 1
    if self.handside == 'leftPalm':
      rs = 0
    ex = 0
    ref = 1
    print "moving to relative position:"
    print position
    self.checkPose()
    newPosition=self.currentPose.position
    newPosition.x+=position.x
    newPosition.y+=position.y
    newPosition.z+=position.z
    linvel = Vector3(0.0,0.0,0.0)
    angvel = Vector3(0.1,0.1,0.1)
    traj = SE3TrajectoryPointRosMessage(time=1.0, position=newPosition, orientation=self.currentPose.orientation, linear_velocity=linvel, angular_velocity=angvel, unique_id=2)
    msg = HandTrajectoryRosMessage(robot_side=rs, taskspace_trajectory_points=[traj], execution_mode=ex, base_for_control=1, unique_id=1)
    # publish message
    self.hand_pub.publish(msg)


  def zeroArm(self):
    print "zeroing arm"
    joint_message = [OneDoFJointTrajectoryRosMessage(trajectory_points = [TrajectoryPoint1DRosMessage(time=3.0, position=0.0, velocity=0.5)] )]*7
    msg = ArmTrajectoryRosMessage(joint_trajectory_messages=joint_message, execution_mode=0, unique_id=13)
    msg.robot_side = 1
    if self.handside == 'leftPalm':
      msg.robot_side = 0
    self.arm_pub.publish(msg)

  def checkPose(self):
    # get current Pose of right Palm, set it to currentPose
    try:
      pose = self.listener.lookup_transform('world', self.handside, rospy.Time(0))
      self.currentPose.position.x = pose.transform.translation.x
      self.currentPose.position.y = pose.transform.translation.y
      self.currentPose.position.z = pose.transform.translation.z
      self.currentPose.orientation.x = pose.transform.rotation.x
      self.currentPose.orientation.y = pose.transform.rotation.y
      self.currentPose.orientation.z = pose.transform.rotation.z
      self.currentPose.orientation.w = pose.transform.rotation.w
    except tf.LookupException as ex:
      print ex


if __name__ == "__main__":

  # instantiate class
  v = valk()
  time.sleep(2)
  try:
    # v.zeroArm()
    # time.sleep(10)
    # move to home
    # v.moveArmHome()
    # time.sleep(15)
    # v.checkPose()
    v.moveArmRelative(Point(-0.1,0,0))
    time.sleep(10)
  except rospy.ROSInterruptException:
    pass
