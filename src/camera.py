#!/usr/bin/env python

# Julian Wu
# Final Camera Sensor
# May 2023

# Importing libraries
import numpy as np
import cv2
import math
import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import threading
try:
    from queue import Queue
except ImportError:
    from Queue import Queue

CAMERA_TOPIC = "/camera/rgb/image_raw"
CMD_VEL_TOPIC = "cmd_vel"
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = 0.2 # rad/s

FREQUENCY = 10

class Camera():
    def __init__(self):

        # Subscribers
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self._camera_callback)

        # Publishers
        self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

        # Other instance variables
        self.rate = rospy.Rate(FREQUENCY)
        self.bridge = CvBridge()

    # Inspired from: https://www.youtube.com/watch?v=-YCcQZmKJtY&ab_channel=D%C3%A1vidDud%C3%A1s
    def _camera_callback(self, camera_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(camera_data, desired_encoding="bgr8")
            image = cv_image

            image = cv2.resize(image, (640, 360))

            height, width, c = image.shape

            # Get RGB channels
            R, G, B = self.image2rgb(image)

            redMask = self.thresholdBinary(R, (220, 255))
            stackedMask = np.dstack((redMask, redMask, redMask))
            contourMask = stackedMask.copy()
            crosshairMask = stackedMask.copy()

            (_, contours, hierarchy) = cv2.findContours(redMask.copy(), 1, cv2.CHAIN_APPROX_NONE)
            print(contours)
            # Find the biggest contours
            if len(contours) > 0:
                print("more than 1 contour")
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

                # Make sure there isn't division by 0
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0
                
                # # Show contour and centroid of ball
                # cv2.drawContours(contourMask, contours, -1, (0, 255, 0), 10)
                # cv2.circle(contourMask, (cx, cy), 5, (0, 255, 0), -1)

                # # Show the crosshair and difference from middle point
                # cv2.line(crosshairMask, (cx,0), (cx,height), (0, 0, 255), 10)
                # cv2.line(crosshairMask, (0,cy), (width,cy), (0,0,255), 10)
                # cv2.line(crosshairMask, (int(width/2),0),(int(width/2),height), (255,0,0), 10)

                # Move towards the ball
                if abs(width/2 - cx) > 20: # rotate towards ball
                    print("Turning")
                    if (width/2 > cx):
                        self.move(0, ANGULAR_VELOCITY)
                    else:
                        self.move(0, -ANGULAR_VELOCITY)
                else:
                    print("Moving")
                    self.move(LINEAR_VELOCITY, 0)
            else:
                self.move(0, ANGULAR_VELOCITY)

            # Show image
            cv2.imshow('window', image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            rospy.logerr(e)
            print(e)

    def image2rgb(self, image):
        R = image[:, :, 2]
        G = image[:, :, 1]
        B = image[:, :, 0]

        return R, G, B
    
    # Apply threshold and result a binary image
    def thresholdBinary(self, img, thresh=(220, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255

    # Moves the robot
    def move(self, linear_vel, angular_vel):
        # print("moving")
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self.cmd_pub.publish(twist_msg)

    # Stop the robot
    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0
        self.cmd_pub.publish(twist_msg)

if __name__ == "__main__":
    rospy.init_node("Camera")
    camera = Camera()
    rospy.on_shutdown(camera.stop)

    print("here")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down...")
    
    cv2.destroyAllWindows()