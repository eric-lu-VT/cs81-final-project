#!/usr/bin/env python
# Author: Jordan Kirkbride

import numpy as np
from geometry_msgs.msg import Twist
import rospy
import tf
import math
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
#from geometry_msgs.msg import PoseStamped, Point, PoseArray, Pose
from nav_msgs.msg import Odometry # tracks current position
import numpy as np
#from std_msgs.msg import Header

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = np.reshape(occupancy_grid_data, (height, width)).T
        self.resolution = resolution

    def cell_at(self, x, y):
        return self.grid[y, x]

class Node:
    def __init__(self, x, y, prev, dist):
        self.x = x
        self.y = y
        self.dist = dist
        self.prev = prev
        self.coord = (self.x, self.y)
    

class Plan:
    def __init__(self):
        self.map = None # the variable containing the map.
        self.visited = []
        self.i = 0
        self.laser_sub = rospy.Subscriber('base_scan', LaserScan, self.laser_callback, queue_size=1)
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        self.odometry = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)
        self.linear_velocity = 0.5
        self.angular_velocity = math.pi/4
        self.occupancy_grid = np.full((100, 100), -1)
        self.map_origin_x = -5
        self.map_origin_y = -5

        self.odom_x = 0
        self.odom_y = 0
        self.odometry_theta = 0

    def odom_callback(self, msg):
        # Get x, y from odom and calculate theta

        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odometry_theta = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])[2]
        


    def publish_map(self):
        # create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.info.resolution = .1
        msg.info.width = self.occupancy_grid.shape[0]
        msg.info.height = self.occupancy_grid.shape[1]
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1
        msg.data = self.occupancy_grid.transpose().flatten()

        # publish the map
        self.map_pub.publish(msg)


        print(msg.data)

    def laser_callback(self, msg):
         # get robot position
        x = self.odom_x
        y = self.odom_y

        # get laser scan data
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

            # update the occupancy grid with the laser readings
        for i in range(len(ranges)):
            if ranges[i] >= msg.range_min and ranges[i] <= msg.range_max:
                # calculate the position of the laser reading in the occupancy grid
                laser_x = int((x + ranges[i]*math.cos(self.odometry_theta + angle_min + i*angle_increment))/.1) + 50
                laser_y = int((y + ranges[i]*math.sin(self.odometry_theta + angle_min + i*angle_increment))/.1) + 50
                
                # mark the cells along the laser beam as free (0)
                for j in range(0, int(ranges[i]/0.1)):
                    cell_x = int((x + j*0.1*math.cos(self.odometry_theta + angle_min + i*angle_increment))/.1) + 50
                    cell_y = int((y + j*0.1*math.sin(self.odometry_theta + angle_min + i*angle_increment))/.1) + 50
                    if cell_x >= 0 and cell_x < 100 and cell_y >= 0 and cell_y < 100:
                        self.occupancy_grid[cell_x][cell_y] = 0
                
                # mark the final cell as occupied (100)
                if laser_x >= 0 and laser_x < 100 and laser_y >= 0 and laser_y < 100:
                    self.occupancy_grid[laser_x][laser_y] = 100
        
        self.publish_map()


                
        

    def translate(self, d): 
        rate = rospy.Rate(100)

        # Move forward a distance of d
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(d/self.linear_velocity):
            self.move(self.linear_velocity, 0)
            rate.sleep()
        self.stop()     

    def rotate_rel(self, angle): 
        rate = rospy.Rate(100)

        # Rotate to face the given angle
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(angle/self.angular_velocity):
            self.move(0, self.angular_velocity)
            rate.sleep()
        self.stop()

    def rotate_abs(self, angle):
        rate = rospy.Rate(100)

        # Rotate to face the given angle
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(abs(self.odometry_theta - angle)/self.angular_velocity):
            self.move(0, self.angular_velocity)
            rate.sleep()
        self.stop()
        
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

    def draw_square(self, side_len):
        self.translate(side_len)
        self.rotate_rel(math.pi/2)
        self.translate(side_len)
        self.rotate_rel(math.pi/2)
        self.translate(side_len)
        self.rotate_rel(math.pi/2)
        self.translate(side_len)
        self.rotate_abs(0)

    def go(self):
        while not rospy.is_shutdown():
        # do other stuff...

            # publish the occupancy grid map
            if self.map is not None:
                self.publish_map()

if __name__ == "__main__":
    rospy.init_node("planner")
    p = Plan()
    rospy.sleep(2)
    #p.go
    #p.draw_square(2)
    #p.rotate_abs(0)
    rospy.spin()

