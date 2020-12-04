#!/usr/bin/env python

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Int32, Float32
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3

G_get_redpos=320

class control(object):

	def __init__(self):
		self.pub = rospy.Publisher("ardrone/takeoff", Empty)
		self.pub_velocity = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.get_redpos=rospy.Subscriber('posi',Int32,self.toGrobal_pos)
		delta_xpos=0

	def takeoff(self):
		
		self.pub.publish(Empty())
		rospy.sleep(5.0)

	def toGrobal_pos(self, a):
		global G_get_redpos
		G_get_redpos = a.data
		# to global
		#be a data

	def main(self):
		if 320-G_get_redpos>10:
			delta_xpos=1
		elif 320-G_get_redpos<-10:
			delta_xpos=-1
		else:
			delta_xpos=0

		print("G_get_redpos",G_get_redpos)
		print("delta_xpos",delta_xpos)


		self.pub_velocity.publish(Twist(Vector3(0,delta_xpos,0),Vector3(0,0,0)))


if __name__=='__main__':
	rospy.init_node('moving_node',anonymous=True)
	control=control()

	

	try:
		control.takeoff()
		while 1:
			control.main()
		
	except rospy.ROSInterruptException:
		pass
