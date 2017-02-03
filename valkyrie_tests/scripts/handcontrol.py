#!/usr/bin/env python

#
# A simple Python script control the fingers on Valkyrie
#


import rospy
import time
import sys
from std_msgs.msg import Float64

if __name__ == '__main__':
    robot_side = 0
    controlmode = 0
    print "robot side -- left:0/right:1; finger range -- open:0/close:"
    if len(sys.argv) == 3:
    	controlmode = 1
        print "First argument for which side, Second for open/close control"
    elif len(sys.argv) == 6:
    	controlmode = 2
        print "First argument for which side, Rest five for fingers control"
    elif len(sys.argv) != 6:
        print "Please start with either 2 or 7 arguments"
        sys.exit()

    robot_side = int(sys.argv[1])
    rospy.init_node ("HandController")
    rate = rospy.Rate(10) # 10hz
    robotSideSym = 0

    if robot_side == 0:
    	robotSideSym =-1
    	pubThumbRoll = rospy.Publisher("/leftThumbRoll_position_controller/command", Float64, queue_size = 1)
    	pubThumbPitch = rospy.Publisher("/leftThumbPitch1_position_controller/command", Float64, queue_size = 1)
    	pubIndexFingerPitch = rospy.Publisher("/leftIndexFingerPitch1_position_controller/command", Float64, queue_size = 1)
    	pubMiddleFingerPitch = rospy.Publisher("/leftMiddleFingerPitch1_position_controller/command", Float64, queue_size = 1)
    	pubPinkyPitch = rospy.Publisher("/leftPinkyPitch1_position_controller/command", Float64, queue_size = 1)
    elif robot_side == 1:
    	robotSideSym =1
    	pubThumbRoll = rospy.Publisher("/rightThumbRoll_position_controller/command", Float64, queue_size = 1)
    	pubThumbPitch = rospy.Publisher("/rightThumbPitch1_position_controller/command", Float64, queue_size = 1)
    	pubIndexFingerPitch = rospy.Publisher("/rightIndexFingerPitch1_position_controller/command", Float64, queue_size = 1)
    	pubMiddleFingerPitch = rospy.Publisher("/rightMiddleFingerPitch1_position_controller/command", Float64, queue_size = 1)
    	pubPinkyPitch = rospy.Publisher("/rightPinkyPitch1_position_controller/command", Float64, queue_size = 1)
    else:
    	print "invalid robot side"
    	sys.exit()

    if controlmode == 1:
    	openclosecommand = -1
    	openclosecommand = int(sys.argv[2])
    	
    	if openclosecommand == 0:
    		msg = Float64(data = robotSideSym*0.01)
    		if pubThumbPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubThumbPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubThumbPitch.publish(msg)
    		msg = Float64(data = robotSideSym*0.01)
    		if pubIndexFingerPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubIndexFingerPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubIndexFingerPitch.publish(msg)
    		msg = Float64(data = robotSideSym*0.01)
    		if pubMiddleFingerPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubMiddleFingerPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubMiddleFingerPitch.publish(msg)
    		msg = Float64(data = robotSideSym*0.01)
    		if pubPinkyPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubPinkyPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubPinkyPitch.publish(msg)
    		time.sleep(2)
    		# if pubThumbRoll.get_num_connections() == 0:
    		# 	print "wait for the publisher!!"
    		# 	while pubThumbRoll.get_num_connections() == 0:
    		# 		rate.sleep()
    		# msg = Float64(data = 0.01)
    		# pubThumbRoll.publish(msg)
    		print "open the hand"
    	elif openclosecommand == 1:
    		if pubThumbRoll.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubThumbRoll.get_num_connections() == 0:
    				rate.sleep()
    		msg = Float64(data = 1.5)
    		pubThumbRoll.publish(msg)
    		time.sleep(2)
    		msg = Float64(data = robotSideSym*0.7)
    		if pubThumbPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubThumbPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubThumbPitch.publish(msg)
    		msg = Float64(data = robotSideSym*0.7)
    		if pubIndexFingerPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubIndexFingerPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubIndexFingerPitch.publish(msg)
    		msg = Float64(data = robotSideSym*0.7)
    		if pubMiddleFingerPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubMiddleFingerPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubMiddleFingerPitch.publish(msg)
    		msg = Float64(data = robotSideSym*0.7)
    		if pubPinkyPitch.get_num_connections() == 0:
    			print "wait for the publisher!!"
    			while pubPinkyPitch.get_num_connections() == 0:
    				rate.sleep()
    		pubPinkyPitch.publish(msg)
    		print "close the hand"
    	else:
    		print "invalid open/close command"
    		sys.exit()

    time.sleep(1)  