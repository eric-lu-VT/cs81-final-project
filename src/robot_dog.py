#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.

# Authors: Eric Lu, Jordan Kirkbride, Julian Wu, Wendell Wu
# Based off of starter code provided to class by Prof. Quattrini Li
# Date: 6/3/23

# Import of python modules.
import math  # use of pi.
import random # use of random
import numpy
import sys

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for cmd_vel
from sensor_msgs.msg import LaserScan  # message type for scan
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
import tf  # library for transformations.
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'  # For simulator, 'base_scan'; for real robot, 'scan'
MAP_TOPIC = '/map'
ODOM_TOPIC = '/odom'
BLL_TOPIC = '/base_scan'  # For simulator, 'base_laser_link'; for real robot, 'base_scan'


# Frequency at which the loop operates
FREQUENCY = 10  # Hz.
WAIT_SLEEP_TIME = 2  # how long to sleep before beginning program (s)

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.5  # m/s # default: 0.5
ANGULAR_VELOCITY = 1 * math.pi / 4  # rad/s # default: math.pi / 4

# Grid parameters
OCCUPANCY_GRID_MAX_PROBABILITY = 100  # Max probability in occupancy grid (by ros specs.) Representing a wall/obstacle
OCCUPANCY_GRID_MIN_PROBABILITY = 0  # Min probability in occupancy grid (by ros specs.) Representing free space
OCCUPANCY_GRID_UNKNOWN = 50  # Representing unseen cell in occupancy grid
RESOLUTION = 0.1  # m/cell
WIDTH = 500  # m
HEIGHT = 500  # m


class Grid:
    """
    Constructor
    """
    def __init__(self, width, height, resolution, origin):
        self.grid = numpy.full([height, width], OCCUPANCY_GRID_UNKNOWN)
        self.resolution = resolution
        self.origin = origin
        self.width = width
        self.height = height

    """
    Determines the probability (in range [0, 100]) that coordinate 
    (x, y) in the grid is occupied, or -1 if it is unseen.

    Args:
        x: x coordinate in the grid reference frame
        y: y coordinate in the grid reference frame
    Returns:
        Probability (in range [0, 100]) that coordinate (x, y) in the grid is occupied, or -1 if it is unseen.
    """
    def cell_at(self, x, y):
        return self.grid[y, x]

    """
    Edits the probability (in range [0, 100]) that coordinate (x, y) in the grid is occupied

    Args:
        x: x coordinate in the grid reference frame
        y: y coordinate in the grid reference frame
        new_probability: new probability, in range [0, 100]
    Returns:
        Nothing
    Throws:
        Exception: Throws error if new probability is not in range [0, 100]
    """
    def edit_cell_at(self, x, y, new_probability):
        if new_probability < OCCUPANCY_GRID_MIN_PROBABILITY or new_probability > OCCUPANCY_GRID_MAX_PROBABILITY:
            raise Exception('invalid probability')

        self.grid[y, x] = new_probability


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
        print(msg)

    """
    Class main function
    """
    def main(self):
        rospy.sleep(WAIT_SLEEP_TIME)  # Sleep to calibrate
        rospy.Rate(FREQUENCY).sleep()

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
