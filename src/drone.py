#!/usr/bin/env python

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Int32, Float32

aruco = cv2.aruco 
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

class ar_pick():
    def __init__(self):
        sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.get_image)
        self.bridge = CvBridge()
        self.image_org = None
        self.pub = rospy.Publisher("arMakar",Image,queue_size=1)
        self.x_posi_pub = rospy.Publisher("x_posi",Float32,queue_size=1)
        self.y_posi_pub = rospy.Publisher("y_posi",Float32,queue_size=1)

    def monitor(self,org):
       
        self.pub.publish(self.bridge.cv2_to_imgmsg(org,"bgr8"))

    def get_image(self,img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def arReader(self):
        if self.image_org is None:
            return None
        frame = self.image_org

        Height, Width = frame.shape[:2] 
        print(Height,Width)

        img = cv2.resize(frame,(int(Width),int(Height)))

        corners, ids, rejectedImgPoints = aruco.detectMarkers(img, dictionary) 

        square_points = np.reshape(np.array(corners), (4, -1))

        point  = (square_points[0]+square_points[2])/2

        aruco.drawDetectedMarkers(img, corners, ids, (0,255,0)) 

        if len(point) > 0:
            list(point)
            
            print type(point[0])
            self.x_posi_pub.publish(point[0])
            self.y_posi_pub.publish(point[1])

        else:
            self.x_posi_pub.publish(320)
            self.y_posi_pub.publish(180)
                
            

        

        

        self.monitor(img)

        cv2.waitKey(1) 


if __name__=='__main__':
    rospy.init_node('ar_pick')


    fd = ar_pick()

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        fd.arReader()#rosservice call /ardrone/togglecam 
        rate.sleep()#rosrun image_view image_view image:=/arMakar

