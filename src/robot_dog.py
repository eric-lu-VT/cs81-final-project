#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.
# type: ignore (ignore missing imports for rospy, tf, etc.)

# Authors: Eric Lu, Jordan Kirkbride, Julian Wu, Wendell Wu
# Based off of starter code provided to class by Prof. Quattrini Li
# Date: 2023-06-03


"""PYTHON MODULES"""
import numpy as np
import math
import rospy
import tf

"""Occupancy Grid Map"""
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan

"""CONSTANTS"""
# Topic names
# DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan', simulation is base_scan
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan', simulation is base_scan
DEFAULT_MAP_TOPIC = 'map'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link'
# DEFAULT_LASER_FRAME = 'base_scan'
DEFAULT_LASER_FRAME = 'base_laser_link'

NODE_NAME = 'robotDog'

# Frequency at which the movement loops operate
FREQUENCY = 10 #Hz.

"""
Field of view in radians that is checked by the robot for wall distance.
In this assignment we assume that the robot can check a full 360 degrees and
the wall measurement is the value scanned directly in front of the robot.
In turtlebot3, the scan angles are from 0 to 2pi.
"""
MIN_SCAN_ANGLE_RAD = -45.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +45.0 / 180 * math.pi

# Maximum Range Allowed as Valid Measurement from Laser, Tunable
# MAX_RANGE = 9.5 # meters for simulation

# Grid Definitions
GRID_WIDTH_M = 50.0 # width of the grid in meters
GRID_HEIGHT_M = 50.0 # height of the grid in meters
GRID_RESOLUTION = 0.05 # resolution of the grid in meters/cell

"""A class representing the occupancy grid map. Taken from my PA3 Code, and
added more functionality to enable publishing/manipulation."""
class Grid:
	def __init__(self, width, height, resolution, origin):
		"""Initialize an empty map grid with everything
		marked as -1 (unknown) message."""
		# turn the grid's occupancy data into a numpy array of the correct
		# dimensions
		self.width = width/resolution # width of the map in cells
		self.height = height/resolution # height of the map in cells
		self.grid = np.full((self.height, self.width), -1)
		self.resolution = resolution # resolution of the map in meters/cell
		self.originPose = origin # origin of the grid in map, a pose object
		self.wallsExpanded = False # turns true after calling expandWalls()
	
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
		# row-major order (C-style, which is the default for numpy and the
		# OccupancyGrid message's expected format)
		msg.data = self.grid.flatten()

		return msg

	def cellAt(self, x, y):
		"""Returns the value of the cell at the given coordinates."""
		# assume row-major order
		return self.grid[y, x]
	
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

"""Robot's Main Class for Interacting with Laser Scanner and Map"""
class RobotDog:
	def __init__(self):
		"""set up subscribers and publishers"""
		# publish our occupancy grid to map
		self.mapPub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
		# laser/LiDAR subscription
		self.laserSub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
		# listener for transforms
		self.tfListener = tf.TransformListener()

		"""Parameters"""
		# origin of grid is at the bottom left corner, odom is positioned
		# at the center of the grid
		originPose = Pose() # no orientation needed
		originPose.position.x = -GRID_WIDTH_M/2
		originPose.position.y = -GRID_HEIGHT_M/2
		originPose.position.z = 0
		self.occGrid = Grid(GRID_WIDTH_M, GRID_HEIGHT_M,\
		  GRID_RESOLUTION, originPose)

		self.laserMsg = None # the most recent laser message, type LaserScan
		self.freshLaser = False # flag to indicate if we have a new laser message

		self.trans = None # most recent translation from odom to base_link
		self.rot = None # most recent rotation from odom to base_link

	def _laser_callback(self, msg):
		"""
		Processing of laser message.
		"""
		# store laser message for use by our main while loop
		self.laserMsg = msg
		self.freshLaser = True

		# store the current position of the robot when the message came in
		self.tfListener.waitForTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0), rospy.Duration(4.0))
		(self.trans, self.rot) = self.tfListener.lookupTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0))
	
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

	"""Occupancy Grid Functions"""

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
			self.occGrid.setCellAt(x, y, 0)
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
			self.occGrid.setCellAt(x, y, 0)
			if D > 0:
				x = x + xi
				D = D + (2 * (dx - dy))
			else:
				D = D + (2 * dx)
	
	def fillOccupancyGrid(self):
		"""Helper function/subroutine to fill the occupancy grid with the
		current data from the laser sensor using the Bresenham
		Line Algorithm/Ray Tracing."""
		
		# from base_link to odom coordinates
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
			# mark the occupied cells correctly in the occupancy grid
			if validRange:
				self.occGrid.setCellAt(xGrid, yGrid, 100)
			
	def spin(self):
		"""Main loop."""		
		print("Grid initialized, mapper started.")
		# set the rate
		rate = rospy.Rate(FREQUENCY)
		while not rospy.is_shutdown():
			# check if we have a new laser message
			if self.freshLaser:
				# reset the flag
				self.freshLaser = False

				# fill the occupancy grid with the current laser data
				self.fillOccupancyGrid()

				# publish it
				msg = self.occGrid.getOccupancyGridMsg()
				self.mapPub.publish(msg)
			rate.sleep()
	

if __name__ == "__main__":
	rospy.init_node(NODE_NAME)

	dog = RobotDog()

	rospy.sleep(2)

	# if interrupted, stop the robot
	# rospy.on_shutdown(None)

	try:
		dog.spin()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS Node Interrupted")
