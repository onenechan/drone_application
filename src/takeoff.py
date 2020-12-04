#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Empty

def takeoff():
	pub = rospy.Publisher("ardrone/takeoff", Empty)
	rospy.init_node('takeoff',anonymous=True)
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(Empty())
		rate.sleep()


if __name__=='__main__':

	try:
		takeoff()
	except rospy.ROSInterruptException:
		pass
