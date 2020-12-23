#!/usr/bin/env python

import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Int32
class Red:
    def __init__(self):
        self.lower = np.array([150, 50, 50])
        self.upper = np.array([180, 255, 255])

class red_pick():
    def __init__(self):
        sub = rospy.Subscriber("/ardrone/front/image_raw",Image,self.get_image)
        self.bridge = CvBridge()
        self.image_org = None
        self.pub = rospy.Publisher("red",Image,queue_size=1)
        self.posi_pub = rospy.Publisher("posi",Int32,queue_size=1)

    def monitor(self,rect,org):
        if rect is not None:
            self.pub.publish(self.bridge.cv2_to_imgmsg(org,"bgr8"))

    def get_image(self,img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


    def detectRectOfTargetColor(self,frame, colorObj):

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask = cv2.inRange(hsv, colorObj.lower, colorObj.upper)

        image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        
        rects = []


        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))

        return rects

    def pick_check(self):
        if self.image_org is None:
            return None
        
        frame = self.image_org

        rects_red = self.detectRectOfTargetColor(frame, Red())

        

    
        if len(rects_red) > 0:
            rect = max(rects_red, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            cent = (rect[0:2]*2+rect[2:4])/2
            print(cent)
            self.posi_pub.publish(cent[0])
            

        if len(rects_red) == 0:
            self.monitor(None,frame)
            return None

        r =rects_red[0]
        self.monitor(r,frame)
        return r

    

  

if __name__=='__main__':
    rospy.init_node('red_pick')
    fd = red_pick()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        fd.pick_check()
        rate.sleep()#rosrun image_view image_view image:=/red
