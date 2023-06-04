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
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import threading
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
from nav_msgs.msg import Odometry
import tf.transformations as tf

CAMERA_TOPIC = "/camera/rgb/image_raw"
CMD_VEL_TOPIC = "cmd_vel"
DEFAULT_SCAN_TOPIC = 'scan'

LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = 0.2 # rad/s
MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi

FREQUENCY = 10

class Camera():
    def __init__(self):

        # Subscribers
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self._camera_callback)
        self.odometry = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        
        # Publishers
        self.cmd_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

        # Other instance variables
        self.rate = rospy.Rate(FREQUENCY)
        self.bridge = CvBridge()

        self.linear_velocity = LINEAR_VELOCITY # Constant linear velocity set.
        self.angular_velocity = ANGULAR_VELOCITY # Constant angular velocity set.

        # Current state from odom
        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_theta = 0
        # Variables for the pd controller
        self.kp = 2
        self.kd = 1
        self.current_distance_error = 1000
        self.prev_distance_error = 0
        self.x_error = 0
        self.y_error = 0
        self.ball_x = 9
        self.ball_y = 9
        self.reached_ball = False
        self.threshold_distance = .2
        self.prev_angle_difference = 0
        self.ball_offset = .5
        self.distance_to_ball = 1000
        self.distance_to_start = 1000
        self.start_x = 0
        self.start_y = 0
        # Not sure which one we need
        self.perceived_ball_area = 1
        self.perceived_ball_perimeter = 1
        self.known_ball_size = .105593 #m
        self.distance_from_ball = 1000
        self.focal_length = 0.00304 #m rapsberry pi camera module 2
        self.pd_takeover = False
        
    def _laser_callback(self, msg):
        # print(msg.ranges[0])
        """Processing of laser message."""
        if self.pd_takeover:

            # Accounts for front half of the robot, finds closest point in front of robot to the wall
            rad2deg = 180 / math.pi
            min_index = 360 + int(MIN_SCAN_ANGLE_RAD * rad2deg)
            max_index = int(MAX_SCAN_ANGLE_RAD * rad2deg)
            min1 = min(list(msg.ranges[min_index:359]))
            min2 = min(list(msg.ranges[0:max_index]))
            self.distance_from_ball = min(min1, min2)

            # self.distance_from_ball = min(front_half)
            print("distance from ball with laser: ", self.distance_from_ball)
            self.pd_controller()

    def pd_controller(self):
        # Travel to the ball if not already there
        if not self.reached_ball:
            print("BALL TOO FAR")
            # self.x_error = self.ball_x - (self.odometry_x + self.ball_offset)
            # self.y_error = self.ball_y - (self.odometry_y + self.ball_offset)
            self.x_error = self.distance_from_ball
            self.y_error = 0
            print("x_error:", self.x_error)
            print("y_error:", self.y_error)
            self.distance_to_ball = math.sqrt(self.x_error**2 + self.y_error**2)
            if self.distance_to_ball < self.threshold_distance: # we have arrived at the ball
                self.reached_ball = True
                #self.goal_position = (self.odometry_x + self.goal_offset, self.odometry_y)  # Set the goal position in front of the robot
                self.stop()
            else:
                self.pd_helper(self.x_error, self.y_error)
        else:
            print("BALL REACHED")
            # self.x_error = self.start_x - (self.odometry_x + self.ball_offset)
            # self.y_error = self.start_y - (self.odometry_y + self.ball_offset)
            # self.distance_to_start = math.sqrt(self.x_error**2 + self.y_error**2)
            # if self.distance_to_ball > self.threshold_distance:
            #     self.reached_ball = False
            # self.pd_helper(self.x_error, self.y_error)
            self.stop()

    def pd_helper(self, x_error, y_error):
        # Calculate angle to the goal position
        target_angle = math.atan2(y_error, x_error)
        # Calculate current robot orientation
        current_angle = self.odometry_theta
        # Calculate the angle difference
        angle_difference = target_angle - current_angle
        # Normalize the angle difference to the range [-pi, pi]
        while angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        while angle_difference < -math.pi:
            angle_difference += 2 * math.pi
        # Update the angular velocity based on the angle difference
        w = self.kp * angle_difference + self.kd * ((angle_difference - self.prev_angle_difference) / 0.02)
        w = 0
        self.prev_angle_difference = angle_difference
        # Move the robot forward with the given linear velocity and calculated angular velocity
        self.move(self.linear_velocity, w)
    
    def odom_callback(self, msg):
        # Get x, y from odom and calculate theta
        self.odometry_x = msg.pose.pose.position.x
        self.odometry_y = msg.pose.pose.position.y
        self.odometry_theta = tf.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]

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
            
            if self.pd_takeover:
                # self.pd_controller()
                # self.getDistanceFromBall(contours)

                # Show image
                cv2.imshow('window', image)
                cv2.waitKey(3)
                return


            # print(contours)
            # Find the biggest contours
            if len(contours) > 0:
                # print("more than 1 contour")
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)

            
                # Make sure there isn't division by 0
                if M["m00"] != 0:
                    print("here")
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
                print("cx:", cx)
                if abs(width/2 - cx) > 20: # rotate towards ball
                    print("Turning")
                    if (width/2 > cx):
                        self.move(0, ANGULAR_VELOCITY)
                    else:
                        self.move(0, -ANGULAR_VELOCITY)
                elif self.distance_from_ball >= 0.2:
                    # print("Moving")
                    self.move(LINEAR_VELOCITY, 0)  
                    self.getDistanceFromBall(contours)             
                else:
                    print("RUNNING PD CONTROLLER")
                    self.pd_takeover = True
            else:
                # print("DID NOT DETECT CONTOUR")
                self.move(0, ANGULAR_VELOCITY)

            # Show image
            cv2.imshow('window', image)
            cv2.waitKey(3)

        except CvBridgeError as e:
            rospy.logerr(e)
            print(e)

    def getDistanceFromBall(self, contours):
        cnt = contours[0]
        self.perceived_ball_area = cv2.contourArea(cnt) 
        self.perceived_ball_perimeter = cv2.arcLength(cnt,True) 
        self.distance_from_ball = ((self.known_ball_size * math.pi) * self.focal_length * 100000) / self.perceived_ball_area
        print("distance from ball: ", self.distance_from_ball)

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