#!/usr/bin/env python
# type: ignore (ignore missing imports for rospy, tf, etc.)
# The line above is important so that this file is interpreted with Python when running it.

# Authors: Eric Lu, Jordan Kirkbride, Julian Wu, Wendell Wu
# Based off of starter code provided to class by Prof. Quattrini Li
# Date: 6/3/23

# Import of python modules.
import math  # use of pi.
import random  # use of random
import numpy
import sys

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for cmd_vel
from sensor_msgs.msg import LaserScan  # message type for scan
from nav_msgs.msg import Odometry
import tf  # library for transformations.
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'  # For simulator, 'base_scan'; for real robot, 'scan'
MAP_TOPIC = '/map'
ODOM_TOPIC = '/odom'
# For simulator, 'base_laser_link'; for real robot, 'base_scan'
BLL_TOPIC = '/base_laser_link'


# Frequency at which the loop operates
FREQUENCY = 10  # Hz.
WAIT_SLEEP_TIME = 2  # how long to sleep before beginning program (s)

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.5  # m/s # default: 0.5
ANGULAR_VELOCITY = 1 * math.pi / 4  # rad/s # default: math.pi / 4

# Grid parameters
# Max probability in occupancy grid (by ros specs.) Representing a wall/obstacle
OCCUPANCY_GRID_MAX_PROBABILITY = 100
# Min probability in occupancy grid (by ros specs.) Representing free space
OCCUPANCY_GRID_MIN_PROBABILITY = 0
OCCUPANCY_GRID_UNKNOWN = -1  # Representing unseen cell in occupancy grid
RESOLUTION = 0.1  # m/cell
WIDTH = 500  # m
HEIGHT = 500  # m


"""A class representing the occupancy grid map. Taken from my PA3 Code, and
added more functionality to enable publishing/manipulation."""


class Grid:
  def __init__(self, width, height, resolution, origin):
    """Initialize an empty map grid with everything
    marked as -1 (unknown) message."""
    # turn the grid's occupancy data into a numpy array of the correct
    # dimensions
    self.grid = numpy.full((height, width), -1)
    self.width = width  # width of the map in cells
    self.height = height  # height of the map in cells
    self.resolution = resolution  # resolution of the map in meters/cell
    self.origin = origin  # origin of the grid in map, a pose object
    self.wallsExpanded = False  # turns true after calling expandWalls()

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
    msg.info.origin = self.origin

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
    self.grid[y, x] = value

  def getGridCoordinates(self, x, y):
    """Converts from continuous world coordinates to grid coordinates."""
    # first offset by the origin of the grid, then divide by resolution
    return (int((x - self.origin.position.x) / self.resolution), int((y - self.origin.position.y) / self.resolution))

  def isPointInGrid(self, x, y):
    return (0 <= y < len(self.grid) and 0 <= x < len(self.grid[0]))
  
  """
  Search for frontiers

  Args:
    x_start: Current x position of robot in grid reference frame
    y_start: Current y position of robot in grid reference frame
  Returns:
    list of frontier points
  """
  def getWavefrontPoints(self, x_start, y_start):
    print('Starting wavefront...')
    queue_map = [(x_start, y_start)]
    map_open_list = {} # list of points enqueued by outer BFS
    map_open_list[(x_start, y_start)] = (x_start, y_start)
    map_close_list = {} # list of points dequeued by outer BFS
    frontier_open_list = {}  # list of points enqueued by inner BFS
    frontier_close_list = {}  # list of points dequeued by inner BFS
        
    contiguous_frontiers = [] # final result to return

    while len(queue_map) > 0: # while queue_m is not empty
      r, c = queue_map.pop(0) # p gets DEQUEUE ( queuem )
      if (r, c) in map_close_list: # if p is marked as "Map -Close - List ":
        continue
            
      isFrontierPoint = False
      for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)):
        if self.isPointInGrid(r + dx, c + dy) and self.cellAt(r + dx, c + dy) == -1:
          isFrontierPoint = True
          break
      if isFrontierPoint: # if p is a frontier point
        queue_frontier = [(r, c)]
        new_frontier = []
        frontier_open_list[(r, c)] = (r, c) # mark p as " Frontier -Open - List "

        while len(queue_frontier) > 0:
          u, v = queue_frontier.pop(0) # q gets DEQUEUE ( queuef )
          if (u, v) in map_close_list or (u, v) in frontier_close_list: # if q is marked as {"Map -Close - List "," Frontier -Close - List "}:
            continue
                    
          isSecondFrontierPoint = False 
          for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            if self.isPointInGrid(u + dx, v + dy) and self.cellAt(u + dx, v + dy) == -1:
              isSecondFrontierPoint = True
              break
          if isSecondFrontierPoint: # if q is a frontier point :
            new_frontier.append((u, v))
            for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)): # for all w in neighbors (q):
              w_pt = (u + dx, v + dy) 
              if not w_pt in frontier_open_list and not w_pt in frontier_close_list and not w_pt in map_close_list:
                queue_frontier.append(w_pt)
                frontier_open_list[w_pt] = w_pt # mark w as " Frontier -Open - List "
          frontier_close_list[(u, v)] = (u, v)
                
        contiguous_frontiers.append(new_frontier)
        for pt in new_frontier:
          map_close_list[pt] = pt
            
      for (dx, dy) in ((1, 0), (0, 1), (-1, 0), (0, -1)): # for all v in neighbors (p):
        if not (r + dx, c + dy) in map_open_list and not (r + dx, c + dy) in map_close_list: # if v not marked as {"Map -Open - List ","Map -Close - List "}
          for (dx1, dy1) in ((1, 0), (0, 1), (-1, 0), (0, -1)): # and v has at least one "Map -Open - Space " neighbor :
            if self.cellAt(r + dx + dx1, c + dx + dy1) == 0:
              queue_map.append((r + dx, c + dy))
              map_open_list[(r + dx, c + dy)] = (r + dx, c + dy) # mark v as "Map -Open - List "
              break

      map_close_list[(r, c)] = (r, c) # mark p as "Map -Close - List"
        
    print('Ending wavefront...')
    return contiguous_frontiers

class RobotDog():
    def __init__(self, mode, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        """Constructor."""

        self.selected_mode = mode

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # Setting up odom subscriber
        self._odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, self._odom_callback)
        # Setting up laser subscriber
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        # Setting up occupancy grid publisher
        self._occupancy_grid_pub = rospy.Publisher(MAP_TOPIC, OccupancyGrid, latch=True)

        # Setting up transformation broadcaster, listener.
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # Parameters.
        self.linear_velocity = linear_velocity  # Constant linear velocity set.
        self.angular_velocity = angular_velocity  # Constant angular velocity set.

        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # The variable containing the map
        origin_pose = Pose()
        origin_pose.position.x = -10  # - (WIDTH * RESOLUTION) / 2
        origin_pose.position.y = -10  # - (HEIGHT * RESOLUTION) / 2
        self.map = Grid(WIDTH, HEIGHT, RESOLUTION, origin_pose)

        # State stuff
        self.is_rotating = False

    """
    Sends a ROS Twist() velocity command.

    Args:
        linear_vel: Desired linear velocity (in m/s)
        angular_vel: Desired angular velocity (in rad/s)
    Returns:
        Nothing
    """
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    """
    Stops the robot.

    Args:
        None
    Returns:
        Nothing
    """
    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    """
    Translates the robot (d) meters forward. No rotation.

    Args:
        d: Desired translation amount (in m)
    Returns:
        Nothing
    """
    def translate(self, d):
        forward_duration = d / LINEAR_VELOCITY
        forward_start_time = rospy.get_rostime()
        while rospy.get_rostime() - forward_start_time < rospy.Duration(forward_duration):
            self.move(self.linear_velocity, 0)
            rospy.Rate(FREQUENCY).sleep()

    """
    Rotates the robot (angle) rad in place

    Args:
        angle: Desired rotation amount (in rad)
    Returns:
        Nothing
    """
    def rotate_rel(self, angle):
        self.is_rotating = True
        # if angle > 0, rotate counterclockwise (i.e., positive angular velocity)
        # if angle <= 0, rotate clockwise (i.e., negative angular velocity)
        rotate_dir = 1 if angle > 0 else -1  # 1 if counterclockwise; -1 clockwise
        rotate_duration = abs(angle) / self.angular_velocity
        rotate_start_time = rospy.get_rostime()
        while rospy.get_rostime() - rotate_start_time < rospy.Duration(rotate_duration):
            self.move(0, rotate_dir * self.angular_velocity)
            rospy.Rate(FREQUENCY).sleep()
        self.is_rotating = False

    """
    Odometry callback function; updates bot position relative to odom

    Args:
        msg: Odometry.msg
    Returns:
        Nothing
    """
    def _odom_callback(self, msg):
        """Callback function."""

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        rot_q = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    """
       LaserScan callback function; updates the OccupancyGrid based on the LIDAR information

       Args:
           msg: LaserScan.msg
       Returns:
           Nothing
       """

    def _laser_callback(self, msg):
        if not self.is_rotating:
            """Processing of laser message."""
            mox = self.map.origin.orientation.x
            moy = self.map.origin.orientation.y
            moz = self.map.origin.orientation.z
            mow = self.map.origin.orientation.w
            (map_roll, map_pitch, map_yaw) = tf.transformations.euler_from_quaternion([mox, moy, moz, mow])
            grid_T_odom = numpy.matrix([
                [math.cos(map_yaw), -math.sin(map_yaw), 0, WIDTH / 2],  # Make it so that OccupancyGrid
                [math.sin(map_yaw), math.cos(map_yaw), 0, HEIGHT / 2],  # is not just first quadrant
                [0, 0, 1, self.map.origin.position.z],
                [0, 0, 0, 1]
            ])

            (trans, rot) = self.listener.lookupTransform(ODOM_TOPIC, BLL_TOPIC, rospy.Time(0))
            t = tf.transformations.translation_matrix(trans)
            R = tf.transformations.quaternion_matrix(rot)
            odom_T_bll = t.dot(R)

            bot_pt_grid = grid_T_odom.dot(numpy.transpose([self.x, self.y, 0, self.map.resolution])) * (
                        1 / self.map.resolution)
            bot_x_grid = numpy.ceil(bot_pt_grid.item(0, 0)).astype(int)
            bot_y_grid = numpy.ceil(bot_pt_grid.item(0, 1)).astype(int)

            # First update the wall positions...
            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + msg.angle_increment * i
                obj_x_bll = r * numpy.cos(angle)
                obj_y_bll = r * numpy.sin(angle)

                obj_pt_odom = odom_T_bll.dot(numpy.transpose([obj_x_bll, obj_y_bll, 0, 1]))
                obj_x_odom = obj_pt_odom.item(0)
                obj_y_odom = obj_pt_odom.item(1)

                obj_pt_grid = grid_T_odom.dot(numpy.transpose([obj_x_odom, obj_y_odom, 0, self.map.resolution])) * (
                            1 / self.map.resolution)
                obj_x_grid = numpy.ceil(obj_pt_grid.item(0, 0)).astype(int)
                obj_y_grid = numpy.ceil(obj_pt_grid.item(0, 1)).astype(int)
                if 0 <= obj_y_grid < len(self.map.grid) and 0 <= obj_x_grid < len(self.map.grid[0]):
                    self.map.setCellAt(obj_x_grid, obj_y_grid, OCCUPANCY_GRID_MAX_PROBABILITY)

            # THEN ray trace
            # The wall positions need to already be set so that the rays can terminate early if needed
            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + msg.angle_increment * i
                obj_x_bll = r * numpy.cos(angle)
                obj_y_bll = r * numpy.sin(angle)

                obj_pt_odom = odom_T_bll.dot(numpy.transpose([obj_x_bll, obj_y_bll, 0, 1]))
                obj_x_odom = obj_pt_odom.item(0)
                obj_y_odom = obj_pt_odom.item(1)

                obj_pt_grid = grid_T_odom.dot(numpy.transpose([obj_x_odom, obj_y_odom, 0, self.map.resolution])) * (
                            1 / self.map.resolution)
                obj_x_grid = numpy.ceil(obj_pt_grid.item(0, 0)).astype(int)
                obj_y_grid = numpy.ceil(obj_pt_grid.item(0, 1)).astype(int)
                if 0 <= obj_y_grid < len(self.map.grid) and 0 <= obj_x_grid < len(self.map.grid[0]):
                    points = self.ray_tracing(bot_x_grid, bot_y_grid, obj_x_grid, obj_y_grid)

                    for (x, y) in points:
                        if 0 <= y < len(self.map.grid) and 0 <= x < len(self.map.grid[0]):
                            # Ray stops early if it hits a wall
                            if self.map.cellAt(x, y) == OCCUPANCY_GRID_MAX_PROBABILITY:
                                break
                            self.map.setCellAt(x, y, OCCUPANCY_GRID_MIN_PROBABILITY)

            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = rospy.Time.now()
            grid_msg.header.frame_id = ODOM_TOPIC  # TODO: Should be odom topic
            grid_msg.info.map_load_time = rospy.Time.now()
            grid_msg.info.resolution = self.map.resolution
            grid_msg.info.width = self.map.width
            grid_msg.info.height = self.map.height
            grid_msg.info.origin = self.map.origin
            grid_msg.data = self.map.grid.flatten()
            self._occupancy_grid_pub.publish(grid_msg)

    """
    Calculates (by brute force) the integer points from (x1, y1) to (x2, y2)

    Args:
        x1: x coordinate of start point
        y1: y coordinate of start point
        x2: x coordinate of end point
        y2: y coordinate of end point

    Returns:
        List of points, in (x, y) format, from (x1, y1) to (x2, y2)
    """
    def ray_tracing(self, x1, y1, x2, y2):
        points = []

        dx = x2 - x1
        dy = y2 - y1
        if dx != 0:  # Account for divide by 0 possibility
            m = dy / dx
            c = y1 - m * x1
            if x1 <= x2:  # Left to right
                for x in range(x1, x2 + 1):
                    y = m * x + c
                    points.append((x, y))
            else:  # Right to left
                for x in range(x1, x2 - 1, -1):
                    y = m * x + c
                    points.append((x, y))

        return points
    
    def wavefront(self):
        mox = self.map.origin.orientation.x
        moy = self.map.origin.orientation.y
        moz = self.map.origin.orientation.z
        mow = self.map.origin.orientation.w
        (map_roll, map_pitch, map_yaw) = tf.transformations.euler_from_quaternion([mox, moy, moz, mow])
        grid_T_odom = numpy.matrix([
          [math.cos(map_yaw), -math.sin(map_yaw), 0, WIDTH / 2],  # Make it so that OccupancyGrid
          [math.sin(map_yaw), math.cos(map_yaw), 0, HEIGHT / 2],  # is not just first quadrant
          [0, 0, 1, self.map.origin.position.z],
          [0, 0, 0, 1]
        ])

        bot_pt_grid = grid_T_odom.dot(numpy.transpose([self.x, self.y, 0, self.map.resolution])) * (1 / self.map.resolution)
        bot_x_grid = numpy.ceil(bot_pt_grid.item(0, 0)).astype(int)
        bot_y_grid = numpy.ceil(bot_pt_grid.item(0, 1)).astype(int)

        res = self.map.getWavefrontPoints(bot_x_grid, bot_y_grid)
        print(len(res))

    """
    Class main function
    """
    def main(self):
        rospy.sleep(WAIT_SLEEP_TIME)  # Sleep to calibrate
        rospy.Rate(FREQUENCY).sleep()

        if self.selected_mode == 1:
            self.translate(1)
            self.wavefront()
            # self.rotate_rel(numpy.pi / 2)
        elif self.selected_mode == 2:   # Select this mode so that all movement is done from the command line
            while not rospy.is_shutdown():
                rospy.Rate(FREQUENCY).sleep()

        self.stop()


"""
Driver function
"""
def main():
  """Main function."""

  # 1st. initialization of node.
  rospy.init_node("robot_dog")

  # Sleep for a few seconds to wait for the registration.
  rospy.sleep(2)

  # Handle mode selection from command line
  if len(sys.argv) < 2:
    print('Invalid number of arguments')
    sys.exit(1)
  arg = sys.argv[1]
  mode = 0
  if arg == '1':
    mode = 1
  elif arg == '2':
    mode = 2
  else:
    print('Unknown argument')
    sys.exit(1)

  # Initialization of the class for the random walk.
  robot_dog = RobotDog(mode)

  # If interrupted, send a stop command before interrupting.
  rospy.on_shutdown(robot_dog.stop)

  # Robot walks.
  try:
    robot_dog.main()
  except rospy.ROSInterruptException:
    rospy.logerr("ROS node interrupted.")


"""
Execute driver
"""
if __name__ == "__main__":
  """Run the main function."""
  main()
