#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollowing():

    def __init__(self):

        self.bridge = CvBridge()
        self.imageSubscriber = rospy.Subscriber("/robot/camera/image_raw", Image, self.cameraCallback)
        self.cmdVelPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.cmdVelRate = rospy.Rate(10)
        self.prevError = 0
    
    def cameraCallback(self, data):

        try:
            cameraImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # The following code is derived from Lab 2
        height, width, channels = cameraImage.shape

        image_hsv = cv2.cvtColor(cameraImage, cv2.COLOR_BGR2HSV)

        upperBound = np.array([115, 255, 255], np.uint8)
        lowerBound = np.array([100,140, 140], np.uint8)
        mask = cv2.inRange(image_hsv, lowerBound, upperBound)
        
        M = cv2.moments(mask)

        if (int(M["m00"]) != 0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            print("Found line!")
        else:
            cX = -1
            cY = height/2
            print("No line found!") 

        #drawnImage = cv2.circle(image_rgb, (cX,cY), 5, (255,0,0), -1)
        
        # PID ALGORITHM
        twist = Twist()
        Kp = 1/200.0
        Kd = 1/75.0
        if (cX != -1):
            error = width/2 - cX
            P = Kp * error
            D = Kd * (error - self.prevError)
            twist.linear.x = 0.4
            twist.angular.z = P + D
            self.prevError = error
        else: 
            twist.linear.x = 0
            twist.angular.z = 0.5

        self.cmdVelPublisher.publish(twist)
        self.cmdVelRate.sleep()  



def main():

    rospy.init_node("line_follow")
    lineFollower = LineFollowing()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    main()