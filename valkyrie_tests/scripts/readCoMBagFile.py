#!/usr/bin/env python

import rospy
import rosbag
from Tkinter import Tk
from tkFileDialog import askopenfilename

if __name__ == '__main__':
	Tk().withdraw()
	filename = askopenfilename()
	print(filename)

	bag = rosbag.Bag(filename)

	file = open("newfile.txt", "w")
	for topic, msg, t in bag.read_messages(topics=['/ihmc_ros/valkyrie/output/capturability/capture_point']):
		# x = msg.x
		# y = msg.y
		# value = (x,y,'\n')
		# s = str(value)
		file.write(str(msg.x))
		file.write(' ')
		file.write(str(-1*msg.y))
		file.write('\n')

	file.close

	bag.close


# Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
# filename = askopenfilename() # show an "Open" dialog box and return the path to the selected file
# print(filename)

# bag = rosbag.Bag('com.bag')
# for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
#     print msg
# bag.close()