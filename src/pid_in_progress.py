#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Jordan Kirkbride '23: Code adapted from Professor Alberto's provided code from pa0 and pa1
# Date Due: 04/21/2023

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
        self.kp = 2
        self.kd = 1
        self.current_distance_error = 1000
        self.prev_distance_error = 0

        self.ball_x = 9
        self.ball_y = 9
        self.reached_ball = False
        self.threshold_distance = .2
        self.prev_angle_difference = 0


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

        # Travel to the ball if not already there
        if not self.reached_ball:
            x_error = self.ball_x - self.odometry_x
            y_error = self.ball_y - self.odometry_y
            print("x_error:", x_error)
            print("y_error:", y_error)

            distance_to_goal = math.sqrt(x_error**2 + y_error**2)
            
            if distance_to_goal < self.threshold_distance: # we have arrived at the ball
                self.reached_ball = True
                #self.goal_position = (self.odometry_x + self.goal_offset, self.odometry_y)  # Set the goal position in front of the robot
                self.stop()
                print("BALL REACHED")
            else:
                self.pd_controller(x_error, y_error, self.linear_velocity)
        else:
            x_error = 0 - self.odometry_x
            y_error = 0 - self.odometry_y
            print("x_error:", x_error)
            print("y_error:", y_error)

            distance_to_goal = math.sqrt(x_error**2 + y_error**2)
            
            if distance_to_goal < self.threshold_distance: # we have arrived at the ball
                self.reached_ball = True
                #self.goal_position = (self.odometry_x + self.goal_offset, self.odometry_y)  # Set the goal position in front of the robot
                self.stop()
            else:
                self.pd_controller(x_error, y_error, self.linear_velocity)

    def pd_controller(self, x_error, y_error, linear_velocity):
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
        self.prev_angle_difference = angle_difference

        # Move the robot forward with the given linear velocity and calculated angular velocity
        self.move(linear_velocity, w)
    
    def odom_callback(self, msg):
        # Get x, y from odom and calculate theta
        self.odometry_x = msg.pose.pose.position.x
        self.odometry_y = msg.pose.pose.position.y
        self.odometry_theta = tf.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]

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