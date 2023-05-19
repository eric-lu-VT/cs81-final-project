#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Authors: Jordan, Eric, Wendall, Julian
# CS81/281 FP3


# Import of python modules.
import math # use of pi.

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # message type for scan
import tf.transformations as tf
from nav_msgs.msg import Odometry # tracks current position

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = .5 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

class FollowWall():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY):
        """Constructor."""

        # Setting up publishers/subscribers.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)
        self.odometry = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
   
        # Current state from odom
        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_theta = 0
        
        # Variables for the pd controller
        self.kp = 1
        self.kd = 2
        self.desired_distance = .5
        self.current_error = 0
        self.prev_error = 0

        # Hold for closest points of right and front of robot to the wall
        self.min_distance_right = 10000
        self.min_distance_front = 10000

        

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
       
        # Accounts for front half of the robot, finds closest point in front of robot to the wall
        front_min_angle = -math.pi/4
        front_max_angle = math.pi/4
        front_min_index = int((front_min_angle - msg.angle_min) / msg.angle_increment)
        front_max_index = int((front_max_angle - msg.angle_min) / msg.angle_increment)
        front_half = msg.ranges[front_min_index:front_max_index+1]
        self.min_distance_front = min(front_half)

        if self.min_distance_front < self.desired_distance: # About to crash from the front, stop and turn
            self.stop()
            self.move(0, math.pi/4)
        else: # Not at goal, calculate error and update angular velocity
            right_min_angle = -math.pi
            right_max_angle = 0
            right_min_index = int((right_min_angle - msg.angle_min) / msg.angle_increment)
            right_max_index = int((right_max_angle - msg.angle_min) / msg.angle_increment)
            right_half = msg.ranges[right_min_index:right_max_index+1]
            self.min_distance_right = min(right_half)
            error = self.desired_distance - self.min_distance_right
            self.pd_controller(error, self.linear_velocity)
            
    def odom_callback(self, msg):
        # Get x, y from odom and calculate theta
        self.odometry_x = msg.pose.pose.position.x
        self.odometry_y = msg.pose.pose.position.y
        self.odometry_theta = tf.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]

    def pd_controller(self, current_error, linear_velocity):
    
        # Calculate error and update angular velocity
        self.prev_error = self.current_error
        self.current_error = current_error
        error_distance =  self.desired_distance - self.min_distance_right
        w = self.kp * error_distance + (self.kd*((self.current_error-self.prev_error)/.02)) 
        self.move(linear_velocity, w)


    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C
            rate.sleep()

def main():
    """Main function."""
  
    # 1st. initialization of node.
    rospy.init_node("follow_wall")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    follow_wall = FollowWall()

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(follow_wall.stop)

    # Robot random walks.
    try:
        follow_wall.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
