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
from geometry_msgs.msg import Pose, Twist, PoseArray
from sensor_msgs.msg import LaserScan

"""CONSTANTS"""
# Topic names
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan', simulation is base_scan
# DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan', simulation is base_scan
DEFAULT_MAP_TOPIC = 'map'
DEFAULT_ODOM_FRAME = 'odom'
DEFAULT_BASE_FRAME = 'base_link'
# DEFAULT_LASER_FRAME = 'base_scan' # this is only for real robot
DEFAULT_LASER_FRAME = 'base_laser_link' # for simulation
# Movement publisher
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'

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

# Velocities that will be used
LINEAR_VELOCITY = 0.22 # m/s - max speed of turtlebot
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Maximum Range Allowed as Valid Measurement from Laser, Tunable
MAX_LASER_RANGE = 9.5 # meters for simulation

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

class Grid:
	"""
	A class representing the occupancy grid map. Taken from Wendell's PA4 
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
	
	def isPointInGrid(self, x, y):
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
						
				contiguous_frontiers.append(new_frontier) # save data of NewFrontier
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
	"""Robot's Main Class"""
	def __init__(self):
		"""set up subscribers and publishers"""
		# movement publisher to cmd_vel
		self.cmdVelPub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
		# publish our occupancy grid to map
		self.occGridPub = rospy.Publisher(DEFAULT_MAP_TOPIC, OccupancyGrid, queue_size=1)
		# laser/LiDAR subscription
		self.laserSub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
		# listener for transforms
		self.tfListener = tf.TransformListener()

		# Robot Parameters
		self.angularVel = ANGULAR_VELOCITY
		self.linearVel = LINEAR_VELOCITY

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

		self.poseArray = None # array of poses for the robot to follow

	def _laser_callback(self, msg):
		"""
		Processing of laser message.
		"""
		# store laser message for use by our main while loop
		self.laserMsg = msg
		self.freshLaser = True

		# store the current position of the robot (laser) when the message came in
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
		
		# from base frame to odom coordinates
		t = tf.transformations.translation_matrix(self.trans)
		R = tf.transformations.quaternion_matrix(self.rot)
		o_T_b = t.dot(R) # base to odom transformation matrix, used to convert

		for i, r in enumerate(self.laserMsg.ranges):
			# ignore invalid measurements
			validRange = True
			if (r > self.laserMsg.range_max) or (r < self.laserMsg.range_min):
				validRange = False
				r = MA

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
		
		# # TODO: Temp

		# ct = [0, 0 ,0]
		# for r in range(len(self.grid)):
		# 	for c in range(len(self.grid[0])):
		# 		if self.grid[r][c] == 0:
		# 			ct[0] += 1
		# 		elif self.grid[r][c] == 100:
		# 			ct[1] += 1
		# 		elif self.grid[r][c] == -1:
		# 			ct[2] += 1
		# print('fillOccupancyGrid [0, 100, -1]: ', ct)
	
	"""Path Planning Functions"""

	def planPath(self, goal):
		"""
		Plan a path to the goal using the occGrid stored in self.occGrid.

		Args:
			goal: a tuple of (x, y) coordinates of the goal in the odom frame
		"""
		# get the current position of the robot in the map frame
		# use self.tfListener to get the transform from map to base_link
		(translation, rotation) = self.tfListener.lookupTransform(DEFAULT_ODOM_FRAME, DEFAULT_BASE_FRAME, rospy.Time(0))
		currentX = translation[0]
		currentY = translation[1]

		# ensure occGrid has walls expanded before planning path
		self.occGrid.expandWalls()

		# use BFS with history to find the path to the goal as a list of (x, y)
		BFSpath = self.BFS((currentX, currentY), goal)
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
		
		# assign it to our member variable
		self.poseArray = path
	
	def BFS(self, start, goal):
		"""Breadth-first search to find the path to the goal."""
		# define start and end as grid indices
		start = self.occGrid.getGridCoordinates(start[0], start[1])
		goal = self.occGrid.getGridCoordinates(goal[0], goal[1])

		print("Planning path from " + str(start) + " to " + str(goal) + "...")

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
		print("No path found to goal: " + str(goal))
		return None

	def followPath(self):
		"""Follow the path stored in self.poseArray. Assumes this is not none."""
		if self.poseArray is None:
			print("No poseArray to follow")
			return
		
		path = self.poseArray.poses
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


	"""Main Functions"""

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
				self.occGridPub.publish(msg)
			rate.sleep()

	def wavefront(self):
		mox = self.occGrid.originPose.orientation.x
		moy = self.occGrid.originPose.orientation.y
		moz = self.occGrid.originPose.orientation.z
		mow = self.occGrid.originPose.orientation.w
		(map_roll, map_pitch, map_yaw) = tf.transformations.euler_from_quaternion([mox, moy, moz, mow])
		grid_T_odom = np.matrix([
			[math.cos(map_yaw), -math.sin(map_yaw), 0, GRID_WIDTH_M / 2],  # Make it so that OccupancyGrid
			[math.sin(map_yaw), math.cos(map_yaw), 0, GRID_HEIGHT_M / 2],  # is not just first quadrant
			[0, 0, 1, self.occGrid.originPose.position.z],
			[0, 0, 0, 1]
		])
			
		t = tf.transformations.translation_matrix(self.trans)
		R = tf.transformations.quaternion_matrix(self.rot)
		o_T_b = t.dot(R) # base to odom transformation matrix, used to convert
		(robotX, robotY) = [int(o_T_b[0,3]), int(o_T_b[1,3])]
		print('Current value of occupancy grid at current robot pos: ', self.occGrid.cellAt(robotX, robotY))

		bot_pt_grid = grid_T_odom.dot(np.transpose([robotX, robotY, 0, self.occGrid.resolution])) * (1 / self.occGrid.resolution)
		bot_x_grid = np.ceil(bot_pt_grid.item(0, 0)).astype(int)
		bot_y_grid = np.ceil(bot_pt_grid.item(0, 1)).astype(int)

		res = self.occGrid.getWavefrontPoints(bot_x_grid, bot_y_grid)
		print(len(res))
	
	def main(self):
		"""
		Driver function
		"""	
		self.translate(0.5)
		self.fillOccupancyGrid()
		# publish it
		msg = self.occGrid.getOccupancyGridMsg()
		self.occGridPub.publish(msg)
		
		self.wavefront()
		self.stop()

if __name__ == "__main__":
	rospy.init_node(NODE_NAME)

	dog = RobotDog()

	rospy.sleep(2)

	rospy.on_shutdown(dog.stop)

	try:
		dog.main()
	except rospy.ROSInterruptException:
		rospy.logerr("ROS Node Interrupted")
