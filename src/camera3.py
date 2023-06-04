#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.
# type: ignore (ignore missing imports for rospy, tf, etc.)

# Authors: Eric Lu, Jordan Kirkbride, Julian Wu, Wendell Wu
# Date: 2023-06-03

# Importing libraries
import numpy as np
import cv2
import math
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, Point, Pose, PoseArray, Point32
from sensor_msgs.msg import Image, LaserScan, PointCloud
from cv_bridge import CvBridge, CvBridgeError
import threading
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
from nav_msgs.msg import Odometry
import tf

CAMERA_TOPIC = "/camera/rgb/image_raw"
CMD_VEL_TOPIC = "cmd_vel"
# TODO: Not sure the difference in scan topic between Stage and Gazebo simulator???
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan', simulation is base_scan
# DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan', simulation is base_scan
DEFAULT_MAP_TOPIC = 'map'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link' # base_footprint
# DEFAULT_LASER_FRAME = 'base_scan' # this is only for real robot
DEFAULT_LASER_FRAME = 'base_laser_link' # for simulation
# Movement publisher
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_POINT_CLOUD_TOPIC = 'point_cloud'

NODE_NAME = 'robotDog'

# Frequency at which the movement loops operate
FREQUENCY = 10 #Hz.

"""
Field of view in radians that is checked by the robot for wall distance.
In this assignment we assume that the robot can check a full 360 degrees and
the wall measurement is the value scanned directly in front of the robot.
In turtlebot3, the scan angles are from 0 to 2pi.
"""
MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi

# Velocities that will be useds
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = 0.2 # rad/s

# Maximum Range Allowed as Valid Measurement from Laser, Tunable
MAX_LASER_RANGE = 9.5 # meters for simulation

ROBOT_SIZE = 0.2 # roughly .2m for turtlebot (defined from tutorial.inc)

# Grid Definitions
# Grid parameters
# Max probability in occupancy grid (by ros specs.) Representing a wall/obstacle
OCCUPANCY_GRID_OCCUPIED = 100
# Min probability in occupancy grid (by ros specs.) Representing free space
OCCUPANCY_GRID_FREE_SPACE = 0
OCCUPANCY_GRID_UNKNOWN = -1  # Representing unseen cell in occupancy grid
GRID_WIDTH_M = 150.0 # width of the grid in meters
GRID_HEIGHT_M = 150.0 # height of the grid in meters
GRID_RESOLUTION = 0.05 # resolution of the grid in meters/cell
MIN_FRONTIER_SIZE = 50 # wrt map reference frame
CLOSE_DIST_THRESHOLD = 15 # wrt odom reference frame, maximum distance (in m) between points to consider them the "same"

# states
STATE_IDLE = 0
STATE_MOVING = 1

FREQUENCY = 10

class Grid:
    """
    A class representing the occupancy grid map. Mostly taken from Wendell's PA4 
    Code, and added more functionality to enable publishing/manipulation.
    """
    def __init__(self, width, height, resolution, origin):
        """Initialize an empty map grid with everything
        marked as -1 (unknown) message."""
        # turn the grid's occupancy data into a np array of the correct
        # dimensions
        self.width = int(width / resolution) # width of the map in cells
        self.height = int(height / resolution) # height of the map in cells
        self.grid = np.full((self.height, self.width), -1)
        self.expandedGrid = np.full((self.height, self.width), -1) # keep second copy just for expanded grid
        self.resolution = resolution # resolution of the map in meters/cell
        self.originPose = origin # origin of the grid in map, a pose object

        self.target = (0, 0)
    
    def getOccupancyGridMsg(self):
        """Return the occupancy grid as a ROS OccupancyGrid message."""
        # create the message
        msg = OccupancyGrid()

        # fill in the metadata
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        # our map's base frame is actually odom, not map
        msg.header.frame_id = DEFAULT_ODOM_FRAME

        msg.info = MapMetaData()
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.resolution = self.resolution
        msg.info.origin = self.originPose

        # convert the grid to a 1D array and fill in the data
        # row-major order (C-style, which is the default for np and the
        # OccupancyGrid message's expected format)
        msg.data = self.grid.flatten()

        return msg

    def cellAt(self, x, y):
        """Returns the value of the cell at the given coordinates."""
        if not self.isPointInGrid(x, y): # out of grid boundaries
            return OCCUPANCY_GRID_OCCUPIED
        # assume row-major order
        return self.grid[y, x]
    
    def cellAtExpanded(self, x, y):
        """Returns the value of the cell at the given coordinates."""
        # assume row-major order
        return self.expandedGrid[y, x]
    
    def setCellAt(self, x, y, value):
        """
        Sets the value of the cell at the given coordinates.
        Values:
        - 0: free
        - 100: occupied
        - -1: unknown
        """
        self.grid[y, x] = value
    
    def getGridCoordinates(self, x, y):
        """Converts from continuous world coordinates to grid coordinates."""
        # first offset by the origin of the grid, then divide by resolution
        return (int((x - self.originPose.position.x) / self.resolution),\
            int((y - self.originPose.position.y) / self.resolution))

    def getWorldCoordinates(self, x, y):
        """
        Converts from grid coordinates to continuous world coordinates.
        Author: Eric Lu
        """
        return (int(x * self.resolution + self.originPose.position.x),\
            int(y * self.resolution + self.originPose.position.y))
    
    def isPointInGrid(self, x, y):
        "Author: Eric Lu"
        return (0 <= y < len(self.grid) and 0 <= x < len(self.grid[0]))
    
    def expandWalls(self):
        """
        Expand the occupied grid cells to account
        for the size of the robot.

        Guarantees that self.expandedGrid is a copy of the most
        recent self.grid with the walls expanded by the size of the robot.
        """
        # how much to expand the walls by, based on the size of the robot and
        # the resolution of the map. units are in cells, ceil to round up.
        expansion = int(math.ceil(ROBOT_SIZE / self.resolution))
        expandedGrid = np.copy(self.grid) # need to edit new copy

        # loop through every cell in the grid currently
        for row in range(len(self.grid)):
            for col in range(len(self.grid[0])):
                # if the cell is occupied in the original grid
                if self.grid[row, col] == OCCUPANCY_GRID_OCCUPIED:
                    # loop through the neighbors
                    for rowsToExpand in range(-expansion, expansion+1):
                        for colsToExpand in range(-expansion, expansion+1):
                            # make sure the indices are valid, don't expand out
                            # of bounds of the grid
                            new_r = row + rowsToExpand
                            new_c = col + colsToExpand
                            if new_r >= 0 and new_r < len(self.grid) and new_c >= 0 and new_c < len(self.grid[0]):
                                # mark it as occupied in the new grid
                                expandedGrid[new_r, new_c] = OCCUPANCY_GRID_OCCUPIED
        self.expandedGrid = expandedGrid

    def getWavefrontPoints(self, x_start, y_start):
        """
        Search for frontiers
        Based on https://cw.fel.cvut.cz/b192/_media/courses/aro/tutorials/04_exploration.pdf
        Author: Eric Lu

        Args:
            x_start: Current x position of robot in grid reference frame
            y_start: Current y position of robot in grid reference frame
        Returns:
            list of frontier points
        """
        queue_map = [(x_start, y_start)]
        map_open_list = {} # list of points enqueued by outer BFS
        map_open_list[(x_start, y_start)] = (x_start, y_start)
        map_close_list = {} # list of points dequeued by outer BFS
        frontier_open_list = {}  # list of points enqueued by inner BFS
        frontier_close_list = {}  # list of points dequeued by inner BFS
        contiguous_frontiers = [] # final result to return

        while len(queue_map) > 0: # while queue_m is not empty
            p = queue_map.pop(0) # p gets DEQUEUE ( queuem )
            #print('p: ', p, self.cellAt(p[0], p[1]))
            if p in map_close_list: # if p is marked as "Map -Close - List ":
                continue
                    
            isFrontierPoint = False
            for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                if self.cellAt(p[0], p[1]) == OCCUPANCY_GRID_UNKNOWN and self.cellAt(p[0] + dx, p[1] + dy) == OCCUPANCY_GRID_FREE_SPACE:
                    isFrontierPoint = True
                    break
            if isFrontierPoint: # if p is a frontier point
                queue_frontier = [p]
                new_frontier = []
                frontier_open_list[p] = p # mark p as " Frontier -Open - List "

                while len(queue_frontier) > 0: # while queue_f is not empty:
                    q = queue_frontier.pop(0) # q gets DEQUEUE ( queue_f )
                    # print('q: ', q, self.cellAt(q[0], q[1]))
                    if q in map_close_list or q in frontier_close_list: # if q is marked as {"Map -Close - List "," Frontier -Close - List "}:
                        continue
                    isSecondFrontierPoint = False 
                    for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                        if self.cellAt(q[0], q[1]) == OCCUPANCY_GRID_UNKNOWN and self.cellAt(q[0] + dx, q[1] + dy) == OCCUPANCY_GRID_FREE_SPACE:
                            isSecondFrontierPoint = True
                            break
                    if isSecondFrontierPoint: # if q is a frontier point :
                        new_frontier.append(q) # NewFrontier <-- q
                        for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)): # for all w in neighbors (q):
                            w_pt = (q[0] + dx, q[1] + dy) 
                            if not w_pt in frontier_open_list and not w_pt in frontier_close_list and not w_pt in map_close_list:
                                queue_frontier.append(w_pt)
                                frontier_open_list[w_pt] = w_pt # mark w as " Frontier -Open - List "
                    frontier_close_list[q] = q # mark q as frontier close list
                        
                if len(new_frontier) > MIN_FRONTIER_SIZE:
                    contiguous_frontiers.append(new_frontier) # save data of NewFrontier, but only if it is large enough
                for pt in new_frontier:
                    map_close_list[pt] = pt # mark all points of NewFrontier as "Map -Close - List"
                    
            for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)): # for all v in neighbors (p):
                v = (p[0] + dx, p[1] + dy)
                isNeighborOpenSpace = False
                for (dx1, dy1) in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                    neighbor_v = (v[0] + dx1, v[1] + dy1)
                    # print('neighbor_v: ', neighbor_v, self.cellAt(neighbor_v[0], neighbor_v[1]))
                    if self.cellAt(neighbor_v[0], neighbor_v[1]) == OCCUPANCY_GRID_FREE_SPACE:
                        isNeighborOpenSpace = True
                        break
                if not v in map_open_list and not v in map_close_list and isNeighborOpenSpace:
                    queue_map.append(v)
                    map_open_list[v] = v # mark v as "Map -Open - List "

            map_close_list[p] = p # mark p as "Map -Close - List"

        print('Ending wavefront...')
        return contiguous_frontiers

class RobotDog:
    def __init__(self):
        """set up subscribers and publishers"""
        # movement publisher to cmd_vel
        self.cmdVelPub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # publish our occupancy grid to map
        self.occGridPub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
        # publish PointCloud
        self.pointCloudPub = rospy.Publisher(DEFAULT_POINT_CLOUD_TOPIC, PointCloud, queue_size=1)
        # laser/LiDAR subscription
        self.laserSub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        # listener for transforms
        self.tfListener = tf.TransformListener()
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self._camera_callback)
        self._odom_sub = rospy.Subscriber(DEFAULT_ODOM_FRAME, Odometry, self._odom_callback, queue_size=1)
        
        """Robot Parameters"""
        self.angularVel = ANGULAR_VELOCITY
        self.linearVel = LINEAR_VELOCITY

        """Grid Parameters"""
        # origin of grid is at the bottom left corner, odom is positioned
        # at the center of the grid
        originPose = Pose() # no orientation needed
        originPose.position.x = -GRID_WIDTH_M/2
        originPose.position.y = -GRID_HEIGHT_M/2
        originPose.position.z = 0
        self.occGrid = Grid(GRID_WIDTH_M, GRID_HEIGHT_M,\
        GRID_RESOLUTION, originPose)

        self.laserMsg = None # the most recent laser message, type LaserScan
        self.freshLaser = True # flag to indicate if we have a new laser message

        self.trans = None # most recent translation from odom to base_link
        self.rot = None # most recent rotation from odom to base_link

        self.frontierCenters = []

        self.state = STATE_IDLE # current state of the robot

        """Other instance variables"""
        self.rate = rospy.Rate(FREQUENCY)
        self.bridge = CvBridge()

        """PD controller variables"""
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
        """Processing of laser message."""
        # store laser message for use by our main while loop
        self.laserMsg = msg
        self.freshLaser = True

        # store the current position of the robot (laser) when the message came in
        self.tfListener.waitForTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0), rospy.Duration(4.0))
        (self.trans, self.rot) = self.tfListener.lookupTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0))
        
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
        self.move(self.linearVel, w)
    
    def _odom_callback(self, msg):
        # Get x, y from odom and calculate theta
        self.odometry_x = msg.pose.pose.position.x
        self.odometry_y = msg.pose.pose.position.y
        self.odometry_theta = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]

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
                    # print("here")
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
                    print("Moving")
                    self.move(LINEAR_VELOCITY, 0)  
                    self.getDistanceFromBall(contours)             
                else:
                    print("RUNNING PD CONTROLLER")
                    self.pd_takeover = True
            else:
                # print("DID NOT DETECT CONTOUR")
                # self.move(0, ANGULAR_VELOCITY)
                delay = 0

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

    """Movement Functions"""

    def stop(self):
        """Stop the robot."""
        self.move(0, 0)

    def move(self, linearVel, angularVel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twistMsg = Twist()

        twistMsg.linear.x = linearVel
        twistMsg.angular.z = angularVel
        self.cmdVelPub.publish(twistMsg)

    def moveForDuration(self, linearVel, angularVel, dur):
        rate = rospy.Rate(FREQUENCY)
        # publish move command for certain time
        startTime = rospy.Time.now()
        while rospy.Time.now() - startTime < rospy.Duration(dur)\
            and not rospy.is_shutdown():
            self.move(linearVel, angularVel)
            rate.sleep()
        self.stop()
        
    def translate(self, distance):
        """Move the robot a certain distance in meters.
        Positive distance is forward, negative is backward."""
        moveTime = abs(distance / self.linearVel)
        direction = 1 if distance > 0 else -1
        self.moveForDuration(self.linearVel * direction, 0, moveTime)

    def rotateRelative(self, angle):
        """Rotate the robot by a certain angle in radians from its current
        angle. Could be positive or negative."""
        rotateTime = abs(angle / self.angularVel)
        direction = 1 if angle > 0 else -1
        self.moveForDuration(0, self.angularVel * direction, rotateTime)
        
    def fillEmptyCellsBetween(self, x0, y0, x1, y1):
        """Directly translated from the Bresenham Line Algorithm pseudocode
        provided on Wikipedia for all cases. Perform a single ray trace and
        marks all cells between x0, y0 and x1, y1 as unoccupied (0)."""
        if abs(y1 - y0) < abs(x1 - x0):
            if x0 > x1:
                self.fillLineLow(x1, y1, x0, y0)
            else:
                self.fillLineLow(x0, y0, x1, y1)
        else:
            if y0 > y1:
                self.fillLineHigh(x1, y1, x0, y0)
            else:
                self.fillLineHigh(x0, y0, x1, y1)
        
    def fillLineLow(self, x0, y0, x1, y1):
        # Called by fillEmptyCellsBetween()
        dx = x1 - x0
        dy = y1 - y0
        yi = 1
        if dy < 0:
            yi = -1
            dy = -dy
        D = (2 * dy) - dx
        y = y0

        # stop before the last cell, which is the end point and should be occupied
        for x in range(x0, x1):
            self.occGrid.setCellAt(x, y, OCCUPANCY_GRID_FREE_SPACE)
            if D > 0:
                y = y + yi
                D = D + (2 * (dy - dx))
            else:
                D = D + (2 * dy)
    
    def fillLineHigh(self, x0, y0, x1, y1):
        # Called by fillEmptyCellsBetween()
        dx = x1 - x0
        dy = y1 - y0
        xi = 1
        if dx < 0:
            xi = -1
            dx = -dx
        D = (2 * dx) - dy
        x = x0

        # stop before the last cell, which is the end point and should be occupied
        for y in range(y0, y1):
            self.occGrid.setCellAt(x, y, OCCUPANCY_GRID_FREE_SPACE)
            if D > 0:
                x = x + xi
                D = D + (2 * (dx - dy))
            else:
                D = D + (2 * dx)
        
    def fillOccupancyGrid(self):
        """Helper function/subroutine to fill the occupancy grid with the
        current data from the laser sensor using the Bresenham
        Line Algorithm/Ray Tracing."""
        
        print('Starting fillOccupancyGrid....')
        while self.trans == None:
            temp = 0
        
        print('Got trans')
        # from base frame to odom coordinates
        t = tf.transformations.translation_matrix(self.trans)
        R = tf.transformations.quaternion_matrix(self.rot)
        o_T_b = t.dot(R) # base to odom transformation matrix, used to convert

        for i, r in enumerate(self.laserMsg.ranges):
            # ignore invalid measurements
            validRange = True
            if (r > self.laserMsg.range_max) or (r < self.laserMsg.range_min):
                validRange = False
                r = self.laserMsg.range_max

            # get the angle of the laser measurement
            angle = self.laserMsg.angle_min + self.laserMsg.angle_increment * i
            # get the x and y coordinates of the laser measurement
            x, y = r * math.cos(angle), r * math.sin(angle)

            # convert from base_link to odom coordinates
            pointVec = np.array([x,y,0,1])
            pointVecInOdom = o_T_b.dot(pointVec.T)

            # convert from odom coordinates to grid coordinates, also the end point of the line
            (xGrid, yGrid) = self.occGrid.getGridCoordinates(pointVecInOdom[0], pointVecInOdom[1])
            # starting point of the line, current position of the robot
            (x0, y0) = [o_T_b[0,3], o_T_b[1,3]]
            x0Grid, y0Grid = self.occGrid.getGridCoordinates(x0, y0)

            # now use the bresenham line tracing algorithm to fill in the
            # cells between the robot and the wall with unoccupied cells
            self.fillEmptyCellsBetween(x0Grid, y0Grid, xGrid, yGrid)
            # mark the occupied cells correctly in the occupancy grid, only
            # if the range is valid
            if validRange:
                self.occGrid.setCellAt(xGrid, yGrid, OCCUPANCY_GRID_OCCUPIED)
        
    def planPath(self, goal, isGoalInGridRefFrame):
        """
        Plan a path to the goal using the occGrid stored in self.occGrid.

        Args:
            goal: a tuple of (x, y) coordinates of the goal in the odom frame
            isGoalInGridRefFrame: True if goal is in the grid reference frame; False otherwise.
        Returns:
            PoseArray() representing path, or None if no path was found
        """
        # get the current position of the robot in the map frame
        # use self.tfListener to get the transform from map to base_link
        (translation, rotation) = self.tfListener.lookupTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0))
        currentX = translation[0]
        currentY = translation[1]

        # ensure occGrid has walls expanded before planning path
        self.occGrid.expandWalls()

        # use BFS with history to find the path to the goal as a list of (x, y)
        BFSpath = self.BFS((currentX, currentY), goal, isGoalInGridRefFrame)
        if BFSpath == None:
            print("No path found to goal: " + str(goal) + " from current position: " + str((currentX, currentY)))
            return

        # turn the grid coordinates into a list of map coordinates to follow
        # create a PoseArray to store the path
        path = PoseArray()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = DEFAULT_ODOM_FRAME

        # loop through the BFS coordinates, turning each into a Pose object
        for i in range(len(BFSpath)):
            node = BFSpath[i]

            # convert the node to map coordinates
            # create a Pose object with the map coordinates
            pose = Pose()
            pose.position.x = node[0] * self.occGrid.resolution + self.occGrid.originPose.position.x
            pose.position.y = node[1] * self.occGrid.resolution + self.occGrid.originPose.position.y

            # calculate the orientation needed to face the next node
            # all except for the last node
            # use atan2 to get the angle between the two points
            # create a quaternion from the angle
            # set the orientation of the pose to the quaternion
            if i < len(BFSpath) - 1:
                nextNode = BFSpath[i + 1]
                quaternion = tf.transformations.quaternion_from_euler(\
                    0, 0,\
                    math.atan2(nextNode[1] - node[1], nextNode[0] - node[0]))
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
            
            # add the pose to the PoseArray
            path.poses.append(pose)
        
        return path
        
    def BFS(self, start, goal, isGoalInGridRefFrame):
        """Breadth-first search to find the path to the goal."""
        # define start and end as grid indices
        start = self.occGrid.getGridCoordinates(start[0], start[1])
        if not isGoalInGridRefFrame:
            goal = self.occGrid.getGridCoordinates(goal[0], goal[1])
            
        print("Planning path from " + str(start) + " to " + str(goal) + "... ", self.occGrid.cellAt(goal[0], goal[1])) # TODO: Temp

        # maintain a queue of tuples of (parent, node)
        frontier = [(start, None)]
        # use a set to track visited nodes so we don't add them again
        visited = set()
        visited.add(start)

        # while the queue is not empty
        while frontier and not rospy.is_shutdown():
            # get the next node from the queue
            currNodePair = frontier.pop(0)
            currNode = currNodePair[0]

            if currNode == goal: # if the current position is the goal
                # reconstruct the path by following the parents
                path = []
                while currNodePair is not None:
                    path.insert(0, currNodePair[0])
                    currNodePair = currNodePair[1]
                return path # return the path found
            
            """Add the neighbors of the current node, assuming an 8-connected
            grid. The order of neighbors is right, left, up, down, up-right,
            down-left, down-right, up-left in the grid frame."""
            neighbors = [(currNode[0]+1, currNode[1]),\
                (currNode[0]-1, currNode[1]),\
                (currNode[0], currNode[1]+1),\
                (currNode[0], currNode[1]-1),\
                (currNode[0]+1, currNode[1]+1),\
                (currNode[0]-1, currNode[1]-1),\
                (currNode[0]+1, currNode[1]-1),\
                (currNode[0]-1, currNode[1]+1)]

            # for each neighbor
            for neighbor in neighbors:
                # if the neighbor is out of bounds or occupied or already
                # visited, do not add it
                # make sure to only call expanded grid functions
                if neighbor[0] < 0 \
                    or neighbor[0] >= self.occGrid.width \
                    or neighbor[1] < 0 \
                    or neighbor[1] >= self.occGrid.height\
                    or self.occGrid.cellAtExpanded(neighbor[0], neighbor[1]) != 0\
                    or neighbor in visited:
                    continue
                
                # add new path including the neighbor to the queue of paths
                frontier.append((neighbor, currNodePair))
                # add the neighbor to the set of visited nodes
                visited.add(neighbor)

        # if the queue is empty and the goal has not been found, return None
        print("No path from: " + str(start) + " to: " + str(goal))
        return None
        
    def addNewPaths(self, frontiers):
        """
        Author: Eric Lu
        """
        print('Starting addNewPaths...')
        bestPoints = []
        for frontier in frontiers:
            bestPoint = frontier[0]
            lowestSumSq = self.occGrid.width * self.occGrid.height # cannot be more than the number of nodes in the graph
            for p in frontier:
                sumSq = 0
                for q in frontier:
                    sumSq += (q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1]) 
                if sumSq < lowestSumSq:
                    bestPoint = p
                    lowestSumSq = sumSq
    
            bestPoints.append(bestPoint)

        for newGridPoint in bestPoints:
            newWorldPoint = self.occGrid.getWorldCoordinates(newGridPoint[0], newGridPoint[1])
            # print('point, worldPoint: ', point, worldPoint) TODO: Temp
            addFlag = True # Do not add point if it is close enough to other points already in oldFrontierCenters
            for (existingWorldPoint, existingGridPoint) in self.frontierCenters:
                dx = existingWorldPoint[0] - newWorldPoint[0]
                dy = existingWorldPoint[1] - newWorldPoint[1]
                if np.sqrt(dx*dx + dy*dy) < CLOSE_DIST_THRESHOLD:
                    addFlag = False
                    break
            if addFlag:
                # print('adding worldPoint: ', worldPoint) TODO: Temp
                self.frontierCenters.append((newWorldPoint, newGridPoint))
                self.occGrid.setCellAt(newGridPoint[0], newGridPoint[1], OCCUPANCY_GRID_FREE_SPACE) # So that BFS works
        
    def followPath(self, poseArray):
        """Follow the path stored in poseArray"""
        if poseArray is None:
            print("No poseArray to follow")
            return
        
        path = poseArray.poses
        for i in range(len(path)):
            # convert pose to be in base_link frame
            (trans, rot) = self.tfListener.lookupTransform(DEFAULT_BASE_FRAME, DEFAULT_ODOM_FRAME, rospy.Time(0))
            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot)
            baselink_T_map = t.dot(R)
            pose = baselink_T_map.dot(np.array([[path[i].position.x], [path[i].position.y], [0], [1]]))

            # first rotate the robot to point towards the next pose
            angle = math.atan2(pose[1], pose[0])
            self.rotateRelative(angle)

            # then move the robot to the next pose
            distance = math.sqrt(pose[1]**2 + pose[0]**2)
            self.translate(distance)
        
    def getFrontiers(self):
        """
        Author: Eric Lu
        """
        # get the current position of the robot in the map frame
        # use self.tfListener to get the transform from map to base_link
        print('here5')
        (translation, rotation) = self.tfListener.lookupTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0))
        robotOdom = self.occGrid.getGridCoordinates(translation[0], translation[1])
        print('Current value of', robotOdom[0], robotOdom[1], ': ', self.occGrid.cellAt(robotOdom[0], robotOdom[1])) # TODO: Temp

        frontiers = self.occGrid.getWavefrontPoints(robotOdom[0], robotOdom[1])
        print('Number of frontiers: ', len(frontiers)) # TODO: Temp
        self.addNewPaths(frontiers)
        print('Number of frontierCenters: ', len(self.frontierCenters)) # TODO: Temp
        
        pointCloudMessage = PointCloud()
        pointCloudMessage.header.stamp = rospy.Time.now()
        pointCloudMessage.header.frame_id = DEFAULT_ODOM_FRAME
        pointCloudMessage.points = []
        for (odomPoint, gridPoint)  in self.frontierCenters:
            pointMessage = Point32()
            pointMessage.x = odomPoint[0]
            pointMessage.y = odomPoint[1]
            pointMessage.z = 0
            pointCloudMessage.points.append(pointMessage)
            print('pointCloud item: ', pointMessage.x, pointMessage.y) # TODO: Temp
        
        self.pointCloudPub.publish(pointCloudMessage)
        print('Done getFrontiers') # TODO: Temp
        
    def main(self):
        """
        Driver function
        """
        rate = rospy.Rate(FREQUENCY)

        while not rospy.is_shutdown():
            # check if we have a new laser message
            print('here1')
            if self.freshLaser:
                print('here2')
                # reset the flag
                self.freshLaser = False

                # fill the occupancy grid with the current laser data
                self.fillOccupancyGrid()
                print('here3')

                # publish it
                msg = self.occGrid.getOccupancyGridMsg()
                self.occGridPub.publish(msg)
                print('here4')
                self.getFrontiers()
            
            if len(self.frontierCenters) > 0:
                print('here9')
                randIdx = np.random.randint(len(self.frontierCenters))
                (odomPoint, gridPoint) = self.frontierCenters[randIdx]
                del self.frontierCenters[randIdx]
                
                nextPoseArray = self.planPath(gridPoint, True)
                print('length of nextPoseArray: ', len(nextPoseArray.poses)) # TODO: Temp
                self.followPath(nextPoseArray) # hook nextTarget
                print("goal reached")
            else:
                print('Done frontier navigation.') # TODO: Temp
                self.stop()
                break

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    
    dog = RobotDog()
    rospy.sleep(2)
    rospy.on_shutdown(dog.stop)
    
    print('Running program...')

    try:
        dog.main()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down...")
    
    cv2.destroyAllWindows()