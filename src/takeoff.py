#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3

pub_velocity = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

def takeoff():
	pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10)
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(Empty())
		rate.sleep()


if __name__=='__main__':
	rospy.init_node('takeoff',anonymous=True)
	try:
		takeoff()
	except rospy.ROSInterruptException:
		pass
